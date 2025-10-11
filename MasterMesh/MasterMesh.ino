// =================================================================================
//                             HEADER & INCLUDES
// =================================================================================
#include "painlessMesh.h"      // Manages the Wi-Fi mesh network communication.
#include <WiFi.h>              // Required by painlessMesh for Wi-Fi functionality.
#include <vector>              // Standard C++ vector for dynamic arrays.
#include <map>                 // Standard C++ map for key-value pair storage.
#include <algorithm>           // For algorithms like std::find.
#include <NimBLEDevice.h>      // A memory-efficient library for Bluetooth Low Energy (BLE) functionality.
#include <Preferences.h>       // For storing data persistently in the ESP32's non-volatile storage (NVS).
#include <ArduinoJson.h>       // For parsing and serializing JSON data, used heavily for data exchange.
#include <set>                 // Standard C++ set for storing unique elements (used in automation).

// =================================================================================
//                            DEFINITIONS & CONSTANTS
// =================================================================================

// Define the GPIO pins for the second UART interface (Serial2), used to communicate with the host controller (e.g., an ESP8266).
#define TX 17
#define RX 16

// Mesh Network Credentials
#define MESH_PREFIX     "MeshHome"     // Name of the mesh network.
#define MESH_PASSWORD   "2147483647"   // Password for the mesh network.
#define MESH_PORT       5555           // Port for mesh communication.

// BLE Scanning Parameters
#define BLE_SCAN_INTERVAL 100 // How often to start a scan (in ms).
#define BLE_SCAN_PERIOD   100 // How long to listen during each interval (in ms).
#define BLE_SCAN_TIME     5000  // Total duration of a single scan cycle (in ms).
#define BLE_SCAN_COUNT    2     // Number of times to repeat the scan cycle before stopping.

// =================================================================================
//                                  DATA STRUCTURES
// =================================================================================

/**
 * @brief Defines the structure for a custom serial communication protocol.
 * This ensures messages are framed correctly between this gateway and the host controller.
 */
struct SerialMessage {
  char command[17];            // 16-byte command string, null-terminated.
  uint8_t payload[1024];       // Buffer for the message data.
  unsigned int length;         // Length of the payload.
  bool valid;                  // Flag to indicate if the message was received correctly.
};

/**
 * @brief Stores information about a discovered BLE device before it's paired.
 */
struct MeshNode {
  NimBLEAdvertisedDevice device; // The advertised device object from the NimBLE library.
  std::string mac_address;       // The device's MAC address.
  std::string display_name;      // The device's advertised name.
};

// =================================================================================
//                             FUNCTION PROTOTYPES
// =================================================================================
void meshCallback(uint32_t from, String &msg);
void sendSerial(const char cmd[16], uint8_t *payload, unsigned int length);
void sendSerial(const char cmd[16], const char *text);
void sendSerial(const char cmd[16], std::vector<uint32_t> &list);
SerialMessage receiveSerialMessage();
void serialCallback(SerialMessage &msg);
void meshSend(uint32_t mesh_id, const char* cmd, String text);
void triggerStateUpdate(uint32_t nodeId);

// =================================================================================
//                               GLOBAL VARIABLES
// =================================================================================
Preferences pref;                                        // Object to interact with persistent storage.
std::map<std::string, MeshNode> foundNodes;              // Temporarily stores BLE devices found during a scan, keyed by MAC address.
std::map<uint32_t, std::string> nodeStateMap;            // Caches the last known state of each mesh node (as a serialized JSON string), keyed by mesh node ID.
painlessMesh mesh;                                       // The main object for managing the mesh network.
int countTimes = 0;                                      // Counter for BLE scan cycles.
bool is_pairing = false;                                 // A flag to prevent multiple simultaneous pairing attempts.
NimBLEClient *pClient = NimBLEDevice::createClient();    // The BLE client object used to connect to peripheral devices.
DynamicJsonDocument configuration(1024);                 // A JSON document to hold the automation configuration rules.
std::map<uint32_t, std::set<uint32_t>> relayTriggerer;   // An optimized lookup map for automation. Key: sensor node ID, Value: set of relay node IDs triggered by this sensor.

// =================================================================================
//                                  BLE LOGIC
// =================================================================================

/**
 * @brief A callback class that defines actions to take during a BLE scan.
 */
class ScanCallbacks : public NimBLEScanCallbacks {

  /**
   * @brief Called for each BLE device found during the scan.
   * @param advertisedDevice Pointer to the device that was found.
   */
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    
    // Check if the device is advertising our specific service UUID, identifying it as a potential node for our system.
    if (advertisedDevice->isAdvertisingService(NimBLEUUID("c220cb58-0d9f-4405-a55a-da4794291e8f"))) {

      // Extract the MAC address from the manufacturer data.
      std::string addr = advertisedDevice->getManufacturerData();

      // Check if this device has already been found in this scan session to avoid duplicates.
      for (const auto &entry : foundNodes) {
        if (entry.second.mac_address == addr) return; // already stored
      }

      // If it's a new, valid device, store its information in our map.
      NimBLEAdvertisedDevice device = *advertisedDevice;
      foundNodes[addr] = {device, addr, advertisedDevice->getName()};
    }
  }

  /**
   * @brief Called when the BLE scan period ends.
   */
  void onScanEnd(const NimBLEScanResults& results, int reason) override {

    // Prepare a payload to send the list of found nodes back to the host controller via UART.
    size_t payloadSize = 4; // Start with 4 bytes for the node count.
    for (const auto &entry : foundNodes) {
        // Calculate the total size needed for all MAC addresses and names.
        payloadSize += 1 + std::string(entry.second.mac_address).size();      // mac length + mac string
        payloadSize += 1 + entry.second.display_name.size();                  // name length + name string
    }

    // Create a byte vector and a pointer to fill it.
    std::vector<uint8_t> payload(payloadSize);
    uint8_t *ptr = payload.data();

    // Encode the number of found nodes first.
    uint32_t count = foundNodes.size();
    memcpy(ptr, &count, 4);
    ptr += 4;

    // Encode each node's MAC and name with a preceding length byte.
    for (const auto &entry : foundNodes) {
      uint8_t macLen = std::string(entry.second.mac_address).size();
      uint8_t nameLen = entry.second.display_name.size();

      *ptr++ = macLen;
      memcpy(ptr, std::string(entry.second.mac_address).data(), macLen);
      ptr += macLen;

      *ptr++ = nameLen;
      memcpy(ptr, entry.second.display_name.data(), nameLen);
      ptr += nameLen;
    }

    // Send the compiled payload over serial to the host.
    sendSerial("res/scan", payload.data(), payload.size());

    // If we haven't reached the scan limit, start another scan cycle.
    if (countTimes <= BLE_SCAN_COUNT) {
      countTimes++;
      NimBLEDevice::getScan()->start(BLE_SCAN_TIME, false, true);
    }
  }
};

// =================================================================================
//                                  UART LOGIC
// =================================================================================

/**
 * @brief A state machine to parse incoming messages from Serial2 based on a custom protocol.
 * Protocol: [START_BYTE (0x00)] [16-BYTE COMMAND] [PAYLOAD] [END_BYTE (0xFF)]
 * @return A SerialMessage struct. `valid` is true if a complete message was parsed.
 */
SerialMessage receiveSerialMessage() {
    static enum { WAIT_START, WAIT_CMD, WAIT_PAYLOAD, WAIT_END } state = WAIT_START;
    static SerialMessage msg;
    static uint8_t cmdIndex = 0;
    static unsigned int payloadIndex = 0;

    while (Serial2.available()) {
      uint8_t byte = Serial2.read();

      switch (state) {
        case WAIT_START:
          if (byte == 0x00) { // start bit = 0x00
              state = WAIT_CMD; // go to next state
              cmdIndex = 0; // starting index of command field
              payloadIndex = 0; // starting index of payload field
              msg.length = 0; // count message length from 0
              msg.valid = false; // initial as invalid first
          }
          break;

        case WAIT_CMD:
          msg.command[cmdIndex++] = byte; // read each byte and store to command field / count up index of command field
          if (cmdIndex == 16) { // limit command field at 16 byte
              msg.command[16] = '\0';  // Null-terminate
              state = WAIT_PAYLOAD; // if reach 16 byte -> go to next state
          }
          break;

        case WAIT_PAYLOAD:
          if (byte == 0xFF) {  // same as command field but no limit
              msg.valid = true;
              state = WAIT_START;
              return msg;
          } else {
              msg.payload[payloadIndex++] = byte;
              msg.length = payloadIndex;
              state = WAIT_END; // go to next state
          }
          break;

        case WAIT_END:
          if (byte == 0xFF) { // stop bit is 0xFF if found -> message end
              msg.valid = true;
              state = WAIT_START;
              return msg;
          } else {
              msg.payload[payloadIndex++] = byte;
              msg.length = payloadIndex;
          }
          break;
      }
    }

    return SerialMessage{"", {}, 0, false}; // null message if error
}

/**
 * @brief Sends a message over Serial2 using the custom protocol.
 * @param cmd The 16-byte command string.
 * @param payload Pointer to the data payload.
 * @param length The length of the payload.
 */
void sendSerial(const char cmd[16], uint8_t *payload, unsigned int length) {
  Serial2.write(0x00);        // Start marker
  Serial2.write((uint8_t*)cmd, 16);    // 16-byte command
  Serial2.write(payload, length);     // Payload
  Serial2.write(0xFF);          // End marker
}

// Convenience overload for sending a C-string as payload.
void sendSerial(const char cmd[16], const char *text) {
  sendSerial(cmd, (uint8_t*)text, strlen(text));
}

// Convenience overload for sending a vector of uint32_t as payload.
void sendSerial(const char cmd[16], std::vector<uint32_t> &list) {
    size_t payloadSize = list.size() * 4;  // 4 bytes per uint32_t
    uint8_t payload[payloadSize];

    for (size_t i = 0; i < list.size(); i++) {
        uint32_t value = list[i];
        memcpy(&payload[i * 4], &value, 4);  // copy 4 bytes
    }

    sendSerial(cmd, payload, payloadSize);
}

/**
 * @brief Main dispatcher for handling commands received from the host via UART.
 * @param msg The parsed serial message.
 */
void serialCallback(SerialMessage &msg) {
  Serial.println("Receiving serial message...");

  // Command: "req/nodes" - Request a list of currently connected nodes.
  if (strcmp(msg.command, "req/nodes") == 0) {
    pref.begin("nodes", true); // Open preferences in read-only mode.
    String json = pref.isKey("save_nodes") ? pref.getString("save_nodes") : "{}";
    pref.end();

    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, json);

    DynamicJsonDocument result(2048);
    const std::list<uint32_t> nodes = mesh.getNodeList(); // Get currently active nodes from painlessMesh.

    // Filter the saved nodes to include only those that are currently online.
    if (!error) {
      for (JsonPair pair : doc.as<JsonObject>()) {
        JsonObject obj = pair.value().as<JsonObject>();
        uint32_t node_id = obj["node_id"];
        // Check if the node ID from storage exists in the list of active nodes.
        if (std::find(nodes.begin(), nodes.end(), node_id) != nodes.end()) {
          result[pair.key()] = obj;
          result[pair.key()]["state"] = getState(node_id);
          if (result[pair.key()]["node_type"] == "relay") {
            result[pair.key()]["config"] = configuration[String(node_id)];
          }
        }
      }
    }

    String jsonResult;
    serializeJson(result, jsonResult);
    sendSerial("res/nodes", jsonResult == "null" ? "{}" : jsonResult.c_str());
  }

  // Command: "req/scan" - Start a new BLE scan for unpaired devices.
  else if (strcmp(msg.command, "req/scan") == 0) {
    foundNodes.clear(); // Clear results from previous scans.
    countTimes = 0;
    Serial.printf("Scanning for peripherals\n");
    NimBLEDevice::getScan()->start(BLE_SCAN_TIME);
  }

  // Command: "req/pair" - Initiate pairing with a device found during a scan.
  else if (strcmp(msg.command, "req/pair") == 0) {
    // The payload is the MAC address of the device to pair with.
    std::string mac(msg.length, '\0');
    memcpy(mac.data(), msg.payload, msg.length);

    auto it = foundNodes.find(mac);
    if (it == foundNodes.end()) {
      // If the device is not in our `foundNodes` map, pairing fails.
      String res = String("{\"mac_address\":\"") + String(mac.c_str()) + "\",\"info\":\"failed to connect to node.\"}";
      sendSerial("res/pair", res.c_str());
      return;
    }

    MeshNode &node = it->second;

    // Proceed if the device address is valid and we are not already in a pairing process.
    if (!node.device.getAddress().toString().empty() && !is_pairing) {
      is_pairing = true; // Set the pairing flag.

      // Connect to the device via BLE.
      if (pClient->connect(node.device)) {
        // Get the specific service for our mesh network.
        NimBLEUUID serviceID("c220cb58-0d9f-4405-a55a-da4794291e8f");
        NimBLERemoteService *pService = pClient->getService(serviceID);

        if (pService != nullptr) {
          // Get the characteristics for writing the mesh credentials.
          auto gatewayCharacteristic = pService->getCharacteristic("46f42176-8d03-4241-843d-195aea0ea390");
          auto meshSSIDCharacteristic = pService->getCharacteristic("98da85e8-42d6-493f-8bff-5416b686f1d2");
          auto meshPasswordCharacteristic = pService->getCharacteristic("207cd133-188a-4503-8a94-17c6070aadce");
          auto meshPortCharacteristic = pService->getCharacteristic("fac0fc9c-f966-4685-bd92-ed4332481bf4");

          // Write the mesh credentials and gateway ID to the device.
          if (gatewayCharacteristic && gatewayCharacteristic->canWrite())
            gatewayCharacteristic->writeValue(String(mesh.getNodeId()).c_str());
          if (meshSSIDCharacteristic && meshSSIDCharacteristic->canWrite())
            meshSSIDCharacteristic->writeValue(MESH_PREFIX);
          if (meshPasswordCharacteristic && meshPasswordCharacteristic->canWrite())
            meshPasswordCharacteristic->writeValue(MESH_PASSWORD);
          if (meshPortCharacteristic && meshPortCharacteristic->canWrite())
            meshPortCharacteristic->writeValue(String(MESH_PORT).c_str());
          sendSerial("res/pair", (String("{\"mac_address\":\"") + String(mac.c_str()) + "\",\"info\":\"pairing to new node.\"}").c_str());
        } else {
          sendSerial("res/pair", (String("{\"mac_address\":\"") + String(mac.c_str()) + "\",\"info\":\"failed to connect to node.\"}").c_str());
        }
        pClient->disconnect();
      } else {
        is_pairing = false; // Reset flag on connection failure.
        sendSerial("res/pair", (String("{\"mac_address\":\"") + String(mac.c_str()) + "\",\"info\":\"failed to connect to node.\"}").c_str());
      }
    } else {
      sendSerial("res/pair", (String("{\"mac_address\":\"") + String(mac.c_str()) + "\",\"info\":\"failed to connect to node.\"}").c_str());
    }
    
  }

  // Command: "req/unpair" - Tell a node to leave the mesh and forget credentials.
  else if (strcmp(msg.command, "req/unpair") == 0) {
    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);

    uint32_t node_id = static_cast<uint32_t>(std::stoul(message));
    meshSend(node_id, "unpair", ""); // Send the "unpair" command over the mesh.
  }

  // Command: "req/allnodes" - Request a list of ALL nodes ever saved, not just online ones.
  else if (strcmp(msg.command, "req/allnodes") == 0) {
    pref.begin("nodes", true);
    String json = pref.isKey("save_nodes") ? pref.getString("save_nodes") : "{}";
    sendSerial("res/allnodes", json.c_str());
    pref.end();
  }

  // Command: "req/state/update" - Force an update to a node's state.
  else if (strcmp(msg.command, "req/state/update") == 0) {

    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);
    
    DynamicJsonDocument incoming(256);
    DeserializationError error = deserializeJson(incoming, message);
    
    if (!error) {
      uint32_t nodeId = incoming["node_id"];
      updateState(nodeId, incoming["state"], true, false, false);
    }
  }
  
  // Command: "req/mode/update" - Update a sensor node's GPIO operating mode
  else if (strcmp(msg.command, "req/mode/update") == 0) {

    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);
    
    DynamicJsonDocument incoming(256);
    DeserializationError error = deserializeJson(incoming, message);
    
    if (!error) {
      uint32_t nodeId = incoming["node_id"];

      JsonObject obj = incoming["mode"];
      String objStr;
      serializeJson(obj, objStr);

      meshSend(nodeId, "update_mode", objStr);
    }
  }

  else if (strcmp(msg.command, "req/mode/get") == 0) {

    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);
    
    uint32_t node_id = static_cast<uint32_t>(std::stoul(message));
    
    meshSend(node_id, "get_mode", "get_mode");
  }

  // Command: "req/state/get" - Request the cached state of a specific node.
  else if (strcmp(msg.command, "req/state/get") == 0) {

    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);
    
    uint32_t node_id = static_cast<uint32_t>(std::stoul(message));

    DynamicJsonDocument output(512);
    output["node_id"] = node_id;

    DynamicJsonDocument state = getState(node_id);

    output["state"] = state;

    String outputStr;
    serializeJson(output, outputStr);

    sendSerial("res/state/get", outputStr.c_str());
  }

  // Command: "req/config/set" - Set or update the automation configuration.
  else if (strcmp(msg.command, "req/config/set") == 0) {
    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);

    DynamicJsonDocument input(1024);
    DeserializationError error = deserializeJson(input, message);
    
    if (!error) {

      pref.begin("nodes", true);
      String json = pref.isKey("save_nodes") ? pref.getString("save_nodes") : "{}";
      DynamicJsonDocument saveNodes(2048);
      DeserializationError error2 = deserializeJson(saveNodes, json);
      pref.end();

      if (!error2) {
        // Iterate through the provided configuration rules.
        for (JsonPair entry : input.as<JsonObject>()) {
          String nodeId = String(entry.key().c_str());
          
          // Verify that the node ID in the config exists in our list of saved nodes.
          bool found = false;
          for (JsonPair saveNodeEntry : saveNodes.as<JsonObject>()) {
            uint32_t saveNodeID = saveNodeEntry.value()["node_id"];
            if (((uint32_t) nodeId.toInt()) == saveNodeID) {
              found = true;
              break;
            }
          }

          if (found) {
            configuration[nodeId] = entry.value();
            sendSerial("res/config/set", ("{\"node_id\":"+nodeId+",\"status\":\"OK\",\"info\":\"OK\"}").c_str());

            // Save the updated configuration to persistent storage.
            pref.begin("configuration", false);
            String configStr;
            serializeJson(configuration, configStr);
            pref.putString("config", configStr);
            Serial.println(pref.getString("config").c_str());
            pref.end();
          } else {
            sendSerial("res/config/set", ("{\"node_id\":"+nodeId+",\"status\":\"ERROR\",\"info\":\"node not found!\"}").c_str());
          }
        }
      } else {
        sendSerial("res/config/set", "{\"status\":\"ERROR\",\"info\":\"Data format error!\"}");
      }
    } else {
      sendSerial("res/config/set", "{\"status\":\"ERROR\",\"info\":\"Data format error!\"}");
    }
  }

  // Command: "req/config/get" - Request the current automation configuration.
  else if (strcmp(msg.command, "req/config/get") == 0) {
    String configStr;
    serializeJson(configuration, configStr);

    sendSerial("res/config/get", configStr.c_str());
  }

  // Command: "req/config/rem" - Remove an automation configuration for a node.
  else if (strcmp(msg.command, "req/config/rem") == 0) {
    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);

    configuration.remove(message); // Remove from in-memory config.
    sendSerial("res/config/rem", ("{\"node_id\":"+String(message.c_str())+",\"status\":\"OK\",\"info\":\"OK\"}").c_str());

    // Clean up the triggerer map to reflect the change.
    for (auto &entry : relayTriggerer) {
      entry.second.erase(entry.first);
    }

    // Save the modified configuration to persistent storage.
    pref.begin("configuration", false);
    String configStr;
    serializeJson(configuration, configStr);
    pref.putString("config", configStr);
    pref.end();
  }
}

// =====================================================
//                     STATE LOGIC
// =====================================================

/**
 * @brief Central function to update a node's state.
 * @param node_id The ID of the node to update.
 * @param state A JsonObject representing the new state.
 * @param callback If true, send the state update over the mesh to the node and send a response via UART.
 * @param sensor If true, this state update is from a sensor and may trigger automations.
 * @param isAuto If true, the update was triggered by automation, affecting the UART response format.
 */
void updateState(uint32_t node_id, JsonObject state, bool callback, bool sensor, bool isAuto) {

  String stateStr;
  serializeJson(state, stateStr);

  bool same = String(nodeStateMap[node_id].c_str()) == stateStr;
  nodeStateMap[node_id] = stateStr.c_str();  // store serialized JSON

  if (callback) {
    if (isAuto) {
      if (!same) {
        meshSend(node_id, "update_state", stateStr);
        String res = String("{\"node_id\":") + String(node_id) + ",\"state\":"+stateStr+"}";
        sendSerial("auto_update", res.c_str());
      }
    } else {
      meshSend(node_id, "update_state", stateStr);
      String res = String("{\"node_id\":") + String(node_id) + ",\"info\":\"OK\"}";
      sendSerial("res/state/update", res.c_str());
    }
  }

  if (sensor) {
    for (auto &relayID : relayTriggerer[node_id]) {
      triggerStateUpdate(relayID);
    }
  }
}

/**
 * @brief Retrieves the cached state of a node.
 * @param node_id The ID of the node.
 * @return A DynamicJsonDocument containing the state. Returns an empty object on failure.
 */
DynamicJsonDocument getState(uint32_t node_id) {
  DynamicJsonDocument output(512); // give enough space

  auto it = nodeStateMap.find(node_id);
  if (it == nodeStateMap.end()) {
    output.to<JsonObject>();
    return output;
  }

  DeserializationError error = deserializeJson(output, it->second);
  if (error) {
    output.to<JsonObject>();
  }
  return output;
}

// =====================================================
//                     MESH LOGIC
// =====================================================

/**
 * @brief Callback function that handles all incoming messages from the mesh network.
 * Messages use a "command|payload" format.
 * @param from The node ID of the sender.
 * @param msg The received message string.
 */
void meshCallback(uint32_t from, String &msg) {

  int sep = msg.indexOf('|');

  String cmd = msg.substring(0, sep);
  String payload = msg.substring(sep + 1);

  Serial.println("Receiving mesh message...");
  Serial.println(String(from)+": "+cmd+"|"+payload);

  if (cmd == "unpairing") {
    sendSerial("res/unpair", ("{\"node_id\":"+String(from)+",\"mac_address\":\""+payload+"\",\"info\":\"unparing\"}").c_str());
    
    pref.begin("nodes", false);
    String json = pref.isKey("save_nodes") ? pref.getString("save_nodes") : "{}";

    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, json);
    
    if (!error) {
      if (doc.containsKey(payload)) {
        doc.remove(payload);
      }
    }

    String docStr;
    serializeJson(doc, docStr);

    pref.putString("save_nodes", docStr);
    pref.end();

    configuration.remove(String(from));

    for (auto &entry : relayTriggerer) {
      entry.second.erase(entry.first);
    }

    pref.begin("configuration", false);
    
    String configStr;
    serializeJson(configuration, configStr);
    
    pref.putString("config", configStr);
    pref.end();
    
  }
  else if (cmd == "node_info" || cmd == "pair_res_info") {

    String mac_address;
    DynamicJsonDocument nodeJson(256);
    DeserializationError err = deserializeJson(nodeJson, payload);

    if (!err) {
      mac_address = String(nodeJson["mac_address"]);

      pref.begin("nodes", false);
    
      String json = pref.isKey("save_nodes") ? pref.getString("save_nodes") : "{}";

      DynamicJsonDocument doc(2048);
      DeserializationError error = deserializeJson(doc, json);

      if (!error) {

        Serial.println("===========================");
        Serial.println(mac_address);
        Serial.println("===========================");
        doc[mac_address] = nodeJson;

        String docStr;
        serializeJson(doc, docStr);

        pref.putString("save_nodes", docStr);

        // Serial.print("DEBUG :");
        // Serial.println(pref.getString("save_nodes", "NO DATA"));
        pref.end();

        if (cmd == "pair_res_info") {
          nodeJson["info"] = "pair success!";

          String pair_res;
          serializeJson(nodeJson, pair_res);

          pClient->end();
          is_pairing = false;
          foundNodes.erase(mac_address.c_str());
          sendSerial("mesh/node/info", pair_res.c_str());
        }

      }
    }
  }

  else if (cmd == "update_mode_response") {
    String res = String("{\"node_id\":") + String(from) + ",\"info\":\"" + payload + "\"}";
    sendSerial("res/mode/update", res.c_str());
  }

  else if (cmd == "state_request") {

    DynamicJsonDocument state = getState(from);

    String stateStr;
    serializeJson(state, stateStr);
    
    meshSend(from, "state_response", stateStr);
  }

  else if (cmd == "state_update_sensor" || cmd == "state_update_relay") {

    DynamicJsonDocument state(512);
    DeserializationError error = deserializeJson(state, payload);
    
    if (!error) {
      updateState(from, state.as<JsonObject>(), false, cmd == "state_update_sensor", false);
    }
  }

  else if (cmd == "get_mode_response") {
    String res = String("{\"node_id\":") + String(from) + ",\"mode\":" + payload + "}";
    sendSerial("res/mode/get", res.c_str());
  }
}

/**
 * @brief Helper function to send a message to a specific node on the mesh.
 * @param mesh_id The destination node ID.
 * @param cmd The command string.
 * @param text The payload string.
 */
void meshSend(uint32_t mesh_id, const char *cmd, String text) {
  String output = String(cmd) + '|' + String(text);
  mesh.sendSingle(mesh_id, output.c_str());
}

// =====================================================
//                 AUTOMATION LOGIC
// =====================================================

/**
 * @brief Evaluates an automation condition string against a sensor's state.
 * Examples: "temp>25.5", "humidity<=60"
 * @param sensorId The node ID of the sensor.
 * @param condition The condition string to evaluate.
 * @return 1 if true, 0 if false, -1 on error or if condition doesn't apply.
 */
int checkCondition(uint32_t sensorId, String condition) {
  String left, right, op;
  if (condition.indexOf(">=") != -1) {
    op = ">=";
  } else if (condition.indexOf("<=") != -1) {
    op = "<=";
  } else if (condition.indexOf(">") != -1) {
    op = ">";
  } else if (condition.indexOf("<") != -1) {
    op = "<";
  } else if (condition.indexOf("=") != -1) {
    op = "=";
  } else if (condition.indexOf("|") != -1) {
    op = "|";
  }

  int pos = condition.indexOf(op);
  if (pos != -1) {
    left = condition.substring(0, pos);
    right = condition.substring(pos + op.length());
  }

  DynamicJsonDocument state = getState(sensorId);
  if (op == "|") {
    if (state.containsKey("gpio")) {
      bool value = state["gpio"][left];
      return value == right.equalsIgnoreCase("true");
    }
    return -1;
  } else {
    if (state.containsKey("i2c")) {
      if (state["i2c"].containsKey(left)) {
        float value = state["i2c"][left];
        if (op == ">") {
          return value > right.toFloat();
        } else if (op == "<") {
          return value < right.toFloat();
        } else if (op == ">=") {
          return value >= right.toFloat();
        } else if (op == "<=") {
          return value <= right.toFloat();
        } else if (op == "=") {
          return value == right.toFloat();
        } else return -1;
      } else return -1;
    } else return -1;
  }
}

void triggerStateUpdate(uint32_t nodeId) {
  // r1
  uint32_t sensorR1 = configuration[String(nodeId)]["r1"]["node_id"];
  String conditionR1 = configuration[String(nodeId)]["r1"]["condition"];

  Serial.printf("condition r1 : %s\n", conditionR1.c_str());
  int result1 = checkCondition(sensorR1, conditionR1);

  uint32_t sensorR2 = configuration[String(nodeId)]["r2"]["node_id"];
  String conditionR2 = configuration[String(nodeId)]["r2"]["condition"];

  Serial.printf("condition r2 : %s\n", conditionR2.c_str());
  int result2 = checkCondition(sensorR2, conditionR2);

  DynamicJsonDocument state(256);

  DynamicJsonDocument oldState = getState(nodeId);

  state["r1"] = result1 == -1 ? (oldState.containsKey("r1") ? oldState["r1"] : false) : (bool) result1;
  state["r2"] = result2 == -1 ? (oldState.containsKey("r2") ? oldState["r2"] : false) : (bool) result2;

  String stateStr;
  serializeJson(state, stateStr);
  Serial.printf("State Triggered: %s\n", stateStr.c_str());

  if (result1 != -1 || result2 != -1) updateState(nodeId, state.as<JsonObject>(), true, false, true);
}

// =====================================================
//                  SETUP FUNCTION
// =====================================================

void setup() {

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX, TX);

  esp_log_level_set("wifi", ESP_LOG_NONE);

  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);

  pref.begin("configuration", true);
  String configStr = pref.isKey("config") ? pref.getString("config") : "{}";
  Serial.printf("config: %s\n", configStr.c_str());
  DeserializationError error = deserializeJson(configuration, configStr);

  if (!error) {
    for (JsonPair pair : configuration.as<JsonObject>()) {
      uint32_t relayId = strtoul(pair.key().c_str(), nullptr, 10);
      relayTriggerer[pair.value()["r1"]["node_id"].as<uint32_t>()].insert(relayId);
      relayTriggerer[pair.value()["r2"]["node_id"].as<uint32_t>()].insert(relayId);
    }
  }

  pref.end();

  // Searching for new node
  NimBLEDevice::init("");
  NimBLEScan *pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(new ScanCallbacks(), false);
  pScan->setActiveScan(true);
  pScan->setInterval(BLE_SCAN_INTERVAL);
  pScan->setWindow(BLE_SCAN_PERIOD);

  // --- initialize Mesh ---
  // mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);
  mesh.onReceive(&meshCallback);
  mesh.setRoot(true);
  mesh.setContainsRoot(true);
}

// =====================================================
//                  LOOP FUNCTION
// =====================================================

uint64_t lastTime = 0;
void loop() {

  mesh.update();

  // ----- Check for ESP8266 Message -----

  SerialMessage msg = receiveSerialMessage();
  if (msg.valid) serialCallback(msg);

  if (millis() - lastTime >= 1000) {
    lastTime = millis();
    // Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());

    for (uint32_t nodeId : mesh.getNodeList()) {
      if (nodeStateMap.find(nodeId) == nodeStateMap.end()) {
        meshSend(nodeId, "get_state", "");
      }
    }
  }
}