#include "painlessMesh.h"
#include <WiFi.h>
#include <vector>
#include <map>
#include <algorithm>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <set>

#define TX 17
#define RX 16

#define MESH_PREFIX     "MeshHome"
#define MESH_PASSWORD   "2147483647"
#define MESH_PORT       5555

#define BLE_SCAN_INTERVAL 100
#define BLE_SCAN_PERIOD   100
#define BLE_SCAN_TIME     5000
#define BLE_SCAN_COUNT    2

struct SerialMessage {
  char command[17];
  uint8_t payload[1024];
  unsigned int length;
  bool valid;
};

struct MeshNode {
  NimBLEAdvertisedDevice device;
  std::string mac_address;
  std::string display_name;
};

void meshCallback(uint32_t from, String &msg);
void sendSerial(const char cmd[16], uint8_t *payload, unsigned int length);
void sendSerial(const char cmd[16], const char *text);
void sendSerial(const char cmd[16], std::vector<uint32_t> &list);
SerialMessage receiveSerialMessage();
void serialCallback(SerialMessage &msg);
void meshSend(uint32_t mesh_id, const char* cmd, String text);
void triggerStateUpdate(uint32_t nodeId);

Preferences pref;
std::map<std::string, MeshNode> foundNodes;
std::map<uint32_t, std::string> nodeStateMap;
painlessMesh mesh;
int countTimes = 0;
bool is_pairing = false;
NimBLEClient *pClient = NimBLEDevice::createClient();
DynamicJsonDocument configuration(1024);
std::map<uint32_t, std::set<uint32_t>> relayTriggerer;

// =====================================================
//                     BLE LOGIC
// =====================================================

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    
    if (advertisedDevice->isAdvertisingService(NimBLEUUID("c220cb58-0d9f-4405-a55a-da4794291e8f"))) {
      
      //Serial.printf("Advertised Device found: %s\n", advertisedDevice->toString().c_str());
      //Serial.printf("%s\n", advertisedDevice->getAddress().toString().c_str());

      // Serial.print("Found target node: ");
      // Serial.println(advertisedDevice->getAddress().toString().c_str());

      // ----- Check of duplicate node -----
      NimBLEAdvertisedDevice device = *advertisedDevice;
      std::string addr = advertisedDevice->getManufacturerData();
      for (const auto &entry : foundNodes) {
        if (entry.second.mac_address == addr) return; // already stored
      }

      // ----- If not exist then push to list -----
      foundNodes[addr] = {device, addr, advertisedDevice->getName()};
    }
  }

  void onScanEnd(const NimBLEScanResults& results, int reason) override {

    // ----- Send respond back to ESP8266 to send to node-red -----
    size_t payloadSize = 4; // 4 bytes for node count
    for (const auto &entry : foundNodes) {

        payloadSize += 1 + std::string(entry.second.mac_address).size();      // mac length + mac string
        payloadSize += 1 + entry.second.display_name.size();     // name length + name string
    }

    std::vector<uint8_t> payload(payloadSize);
    uint8_t *ptr = payload.data();

    uint32_t count = foundNodes.size();
    memcpy(ptr, &count, 4);
    ptr += 4;

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

    sendSerial("res/scan", payload.data(), payload.size());

    if (countTimes <= BLE_SCAN_COUNT) {
      countTimes++;
      NimBLEDevice::getScan()->start(BLE_SCAN_TIME, false, true);
    }
  }
};

// =====================================================
//                     UART LOGIC
// =====================================================

SerialMessage receiveSerialMessage() {
    static enum { WAIT_START, WAIT_CMD, WAIT_PAYLOAD, WAIT_END } state = WAIT_START;
    static SerialMessage msg;
    static uint8_t cmdIndex = 0;
    static unsigned int payloadIndex = 0;

    while (Serial2.available()) {
      uint8_t byte = Serial2.read();

      // Serial.print((char) byte);

      // Serial.println();

      switch (state) {
        case WAIT_START:
          if (byte == 0x00) {
              state = WAIT_CMD;
              cmdIndex = 0;
              payloadIndex = 0;
              msg.length = 0;
              msg.valid = false;
          }
          break;

        case WAIT_CMD:
          msg.command[cmdIndex++] = byte;
          if (cmdIndex == 16) {
              msg.command[16] = '\0';  // Null-terminate
              state = WAIT_PAYLOAD;
          }
          break;

        case WAIT_PAYLOAD:
          if (byte == 0xFF) {  // No payload
              msg.valid = true;
              state = WAIT_START;
              return msg;
          } else {
              msg.payload[payloadIndex++] = byte;
              msg.length = payloadIndex;
              state = WAIT_END;
          }
          break;

        case WAIT_END:
          if (byte == 0xFF) {
              msg.valid = true;
              state = WAIT_START;
              return msg;
          } else {
              // keep collecting until end marker
              msg.payload[payloadIndex++] = byte;
              msg.length = payloadIndex;
          }
          break;
      }
    }

    return SerialMessage{"", {}, 0, false};
}

void sendSerial(const char cmd[16], uint8_t *payload, unsigned int length) {

  Serial2.write(0x00);        // Start marker
  Serial2.write((uint8_t*)cmd, 16);    // 16-byte command
  Serial2.write(payload, length);     // Payload
  Serial2.write(0xFF);          // End marker
  
}

void sendSerial(const char cmd[16], const char *text) {
  sendSerial(cmd, (uint8_t*)text, strlen(text));
}

void sendSerial(const char cmd[16], std::vector<uint32_t> &list) {
    size_t payloadSize = list.size() * 4;  // 4 bytes per uint32_t
    uint8_t payload[payloadSize];

    for (size_t i = 0; i < list.size(); i++) {
        uint32_t value = list[i];
        memcpy(&payload[i * 4], &value, 4);  // copy 4 bytes
    }

    // Replace this with your actual sending function
    sendSerial(cmd, payload, payloadSize);
}

void serialCallback(SerialMessage &msg) {

  Serial.println("Receiving serial message...");

  if (strcmp(msg.command, "req/nodes") == 0) {

    
    pref.begin("nodes", true);

    String json = pref.isKey("save_nodes") ? pref.getString("save_nodes") : "{}";

    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, json);

    DynamicJsonDocument result(2048);
    const std::list<uint32_t> nodes = mesh.getNodeList();

    // Serial.print("DEBUG : ");
    // for (auto &n : nodes) {
    //   Serial.print(String(n)+" ");
    // }
    // Serial.println();
    if (!error) {
      for (JsonPair pair : doc.as<JsonObject>()) {
        // Serial.print("DEBUG : ");
        // Serial.print(pair.key().c_str());
        JsonObject obj = pair.value().as<JsonObject>();
        uint32_t node_id = obj["node_id"];
        // Serial.println(" | " + String(node_id));
        if (std::find(nodes.begin(), nodes.end(), node_id) != nodes.end()) {
          // Serial.println("DEBUG : Reach condition");
          result[pair.key()] = obj;

          result[pair.key()]["state"] = getState(node_id);
          
          String s = result[pair.key()]["mac_address"];
          // Serial.println("DEBUG : " + s);
        }
      }
    }

    String jsonResult;
    serializeJson(result, jsonResult);

    Serial.println("DEBUG : " + jsonResult);
    
    sendSerial("res/nodes", jsonResult == "null" ? "{}" : jsonResult.c_str());
    pref.end();
  }

  else if (strcmp(msg.command, "req/scan") == 0) {
    foundNodes.clear();
    countTimes = 0;
    Serial.printf("Scanning for peripherals\n");
    NimBLEDevice::getScan()->start(BLE_SCAN_TIME);
  }

  else if (strcmp(msg.command, "req/pair") == 0) {
    // Safely build string from payload



    std::string mac(msg.length, '\0');
    memcpy(mac.data(), msg.payload, msg.length);


    // Find node safely (don’t use operator[] directly)
    auto it = foundNodes.find(mac);

    if (it == foundNodes.end()) {

      String res = String("{\"mac_address\":\"") + String(mac.c_str()) + "\",\"info\":\"failed to connect to node.\"}";
      sendSerial("res/pair", res.c_str());
      return;
    }


    MeshNode &node = it->second;



    if (!node.device.getAddress().toString().empty() && !is_pairing) {

      is_pairing = true;
      if (pClient->connect(node.device)) {

        NimBLEUUID serviceID("c220cb58-0d9f-4405-a55a-da4794291e8f");
        NimBLERemoteService *pService = pClient->getService(serviceID);

        if (pService != nullptr) {

          auto gatewayCharacteristic = pService->getCharacteristic("46f42176-8d03-4241-843d-195aea0ea390");
          auto meshSSIDCharacteristic = pService->getCharacteristic("98da85e8-42d6-493f-8bff-5416b686f1d2");
          auto meshPasswordCharacteristic = pService->getCharacteristic("207cd133-188a-4503-8a94-17c6070aadce");
          auto meshPortCharacteristic = pService->getCharacteristic("fac0fc9c-f966-4685-bd92-ed4332481bf4");

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
        is_pairing = false;
        sendSerial("res/pair", (String("{\"mac_address\":\"") + String(mac.c_str()) + "\",\"info\":\"failed to connect to node.\"}").c_str());
      }
    } else {
      sendSerial("res/pair", (String("{\"mac_address\":\"") + String(mac.c_str()) + "\",\"info\":\"failed to connect to node.\"}").c_str());
    }
    
  }

  else if (strcmp(msg.command, "req/unpair") == 0) {
    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);

    uint32_t node_id = static_cast<uint32_t>(std::stoul(message));

    meshSend(node_id, "unpair", "");
  }

  else if (strcmp(msg.command, "req/allnodes") == 0) {
    
    pref.begin("nodes", true);
    String json = pref.isKey("save_nodes") ? pref.getString("save_nodes") : "{}";
    Serial.print("DEBUG :");
    Serial.println(json);
    sendSerial("res/allnodes", json.c_str());
    pref.end();
  }

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
        for (JsonPair entry : input.as<JsonObject>()) {
          String nodeId = String(entry.key().c_str());
          
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
  else if (strcmp(msg.command, "req/config/get") == 0) {
    
    String configStr;
    serializeJson(configuration, configStr);

    sendSerial("res/config/get", configStr.c_str());

  }
  else if (strcmp(msg.command, "req/config/rem") == 0) {

    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);

    configuration.remove(message);
    sendSerial("res/config/rem", ("{\"node_id\":"+String(message.c_str())+",\"status\":\"OK\",\"info\":\"OK\"}").c_str());

    for (auto &entry : relayTriggerer) {
      entry.second.erase(entry.first);
    }

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
}

void meshSend(uint32_t mesh_id, const char *cmd, String text) {

  String output = String(cmd) + '|' + String(text);

  mesh.sendSingle(mesh_id, output.c_str());
  
}

// =====================================================
//                 AUTOMATION LOGIC
// =====================================================

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