#include <SoftwareSerial.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <vector>
#include <string>

#define NETWORK_NAME "IoT"

#define TX 15  // GPIO15 (D8)
#define RX 13  // GPIO13 (D7)

// Protocol Configuration
#define TOPIC_PING        "esp/ping"
#define TOPIC_PONG        "esp/pong"
#define TOPIC_MESH_NODE_REQUEST       "req/mesh/nodes"
#define TOPIC_MESH_NODE_RESPONSE      "res/mesh/nodes"
#define TOPIC_MESH_ALL_NODE_REQUEST   "req/mesh/allnodes"
#define TOPIC_MESH_ALL_NODE_RESPONSE  "res/mesh/allnodes"
#define TOPIC_MESH_SCAN_REQUEST       "req/mesh/scan"
#define TOPIC_MESH_SCAN_RESPONSE      "res/mesh/scan"
#define TOPIC_MESH_PAIR_REQUEST       "req/mesh/pair"
#define TOPIC_MESH_PAIR_RESPONSE      "res/mesh/pair"
#define TOPIC_MESH_UNPAIR_REQUEST     "req/mesh/unpair"
#define TOPIC_MESH_UNPAIR_RESPONSE    "res/mesh/unpair"
#define TOPIC_STATE_UPDATE_REQUEST    "req/state/update"
#define TOPIC_STATE_UPDATE_RESPONSE   "res/state/update"
#define TOPIC_STATE_GET_REQUEST       "req/state/get"
#define TOPIC_STATE_GET_RESPONSE      "res/state/get"
#define TOPIC_MODE_UPDATE_REQUEST     "req/mode/update"
#define TOPIC_MODE_UPDATE_RESPONSE    "res/mode/update"
#define TOPIC_CONFIGURATION_SET_REQUEST     "req/config/set"
#define TOPIC_CONFIGURATION_SET_RESPONSE    "res/config/set"
#define TOPIC_CONFIGURATION_REMOVE_REQUEST  "req/config/remove"
#define TOPIC_CONFIGURATION_REMOVE_RESPONSE "res/config/remove"
#define TOPIC_CONFIGURATION_GET_REQUEST     "req/config/get"
#define TOPIC_CONFIGURATION_GET_RESPONSE    "res/config/get"
#define TOPIC_STATE_AUTO_UPDATE       "res/state/auto_update"

#define EEPROM_SIZE 50

struct SerialMessage {
  char command[17];
  uint8_t payload[1024];
  unsigned int length;
  bool valid;
};

struct MeshNode {
  std::string mac_address;
  std::string display_name;
};

SoftwareSerial Serial2(RX, TX); // RX=GPIO13, TX=GPIO15

WiFiManager wm;
WiFiClient wifiClient;
PubSubClient pubSubClient(wifiClient);

char mqtt_server[40] = "";

void mqttCallback(char* topic, byte* message, unsigned int length);
void connectMQTT();
void sendSerial(const char cmd[16], uint8_t *payload, unsigned int length);
void sendSerial(const char cmd[16], const char *text);
void sendSerial(const char cmd[16], std::vector<uint32_t> &list);
void serialCallback(SerialMessage &msg);
SerialMessage receiveSerialMessage();

// =====================================================
//                  SETUP FUNCTION
// =====================================================

void setup() {

  Serial.begin(115200);  // Debug
  Serial2.begin(9600);   // Communication with ESP32

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  Serial.println("ESP8266 (Software UART): Ready");

  // ------- Initial EEPROM and READ ---------
  EEPROM.begin(EEPROM_SIZE);
  Serial.println("Success init EEPROM");

  // --- Read MQTT server from EEPROM ---
  if (EEPROM.read(0) != 0x42) {
    Serial.println("No valid data in EEPROM.");
    mqtt_server[0] = '\0';
  } else {
    for (int i = 0; i < 39; i++) {
      mqtt_server[i] = EEPROM.read(i + 1);
      if (mqtt_server[i] == '\0') break;
    }
    mqtt_server[39] = '\0';
  }

  sprintf(mqtt_server, "%s", "192.168.1.150");

  if (strlen(mqtt_server) == 0) {
    wm.resetSettings();  // force config portal if no data
  }

  // ----- Connect to WiFi --------
  WiFiManagerParameter mqttServer("server", "Dashboard Server", mqtt_server, 40);
  wm.addParameter(&mqttServer);

  if (!wm.autoConnect("SmartHub-Setup")) {
    Serial.println("Failed to connect. Restarting...");
    delay(3000);
    ESP.restart();
  }

  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());

  // --- Save MQTT server if changed ---
  strcpy(mqtt_server, mqttServer.getValue());
  EEPROM.write(0, 0x42);
  for (int i = 0; i < 40; i++) EEPROM.write(i + 1, mqtt_server[i]);
  EEPROM.commit();

  Serial.printf("Dashboard Server Info: %s\n", mqtt_server);

  // --- MQTT setup & connect ---
  pubSubClient.setServer(mqtt_server, 1883);
  pubSubClient.setBufferSize(1024);
  pubSubClient.setCallback(mqttCallback);
  connectMQTT();
}

// =====================================================
//                  LOOP FUNCTION
// =====================================================

long long nextTime = 0;
void loop() {

  pubSubClient.loop();

// Check if mqtt disconnect then connect again

  if (millis() >= nextTime) {
    nextTime = millis() + 5000;
    if (!pubSubClient.connected()) {
      connectMQTT();
    }
  }

  SerialMessage msg = receiveSerialMessage();
  if (msg.valid) serialCallback(msg);
}


void mqttCallback(char* topic, byte* message, unsigned int length) {
  digitalWrite(2, HIGH);
  delay(200);
  digitalWrite(2, LOW);

  
  String incoming;
  for (unsigned int i = 0; i < length; i++) incoming += (char)message[i];

  // ----- PING -----
  if (String(topic) == TOPIC_PING) {
    pubSubClient.publish(TOPIC_PONG, "alive");
  }

  // ----- Node Request -----
  else if (String(topic) == TOPIC_MESH_NODE_REQUEST) {
    // ESP32 Communication here
    sendSerial("req/nodes", "request");
  }

  else if (String(topic) == TOPIC_MESH_SCAN_REQUEST) {
    sendSerial("req/scan", "request");
  }

  else if (String(topic) == TOPIC_MESH_PAIR_REQUEST) {
    sendSerial("req/pair", incoming.c_str());
  }

  else if (String(topic) == TOPIC_MESH_UNPAIR_REQUEST) {
    sendSerial("req/unpair", incoming.c_str());
  }

  else if (String(topic) == TOPIC_MESH_ALL_NODE_REQUEST) {
    sendSerial("req/allnodes", incoming.c_str());
  }
  else if (String(topic) == TOPIC_STATE_UPDATE_REQUEST) {
    sendSerial("req/state/update", incoming.c_str());
  }
  else if (String(topic) == TOPIC_STATE_GET_REQUEST) {
    sendSerial("req/state/get", incoming.c_str());
  }
  else if (String(topic) == TOPIC_MODE_UPDATE_REQUEST) {
    sendSerial("req/mode/update", incoming.c_str());
  }
  else if (String(topic) == TOPIC_CONFIGURATION_SET_REQUEST) {
    sendSerial("req/config/set", incoming.c_str());
  }
  else if (String(topic) == TOPIC_CONFIGURATION_GET_REQUEST) {
    sendSerial("req/config/get", incoming.c_str());
  }
  else if (String(topic) == TOPIC_CONFIGURATION_REMOVE_REQUEST) {
    sendSerial("req/config/rem", incoming.c_str());
  }
}

void connectMQTT() {
  Serial.println("Attempting MQTT connection...");
  if (!pubSubClient.connected()) {
    String clientId = "ESP8266-GW-";
    clientId += String(ESP.getChipId(), HEX); // shorter clientId for ESP8266
    if (pubSubClient.connect(clientId.c_str())) {
      pubSubClient.subscribe(TOPIC_PING);
      pubSubClient.subscribe(TOPIC_MESH_NODE_REQUEST);
      pubSubClient.subscribe(TOPIC_MESH_SCAN_REQUEST);
      pubSubClient.subscribe(TOPIC_MESH_PAIR_REQUEST);
      pubSubClient.subscribe(TOPIC_MESH_UNPAIR_REQUEST);
      pubSubClient.subscribe(TOPIC_MESH_ALL_NODE_REQUEST);
      pubSubClient.subscribe(TOPIC_STATE_UPDATE_REQUEST);
      pubSubClient.subscribe(TOPIC_STATE_GET_REQUEST);
      pubSubClient.subscribe(TOPIC_MODE_UPDATE_REQUEST);
      pubSubClient.subscribe(TOPIC_CONFIGURATION_REMOVE_REQUEST);
      pubSubClient.subscribe(TOPIC_CONFIGURATION_GET_REQUEST);
      pubSubClient.subscribe(TOPIC_CONFIGURATION_SET_REQUEST);
      Serial.println("MQTT Connected!");
    } else {
      Serial.println("Failed MQTT connection");
    }
  }
}

// =====================================================
//              SEND & RECEIVE UART PART
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
    Serial.write(cmd, strlen(cmd));
    Serial.write(payload, payloadSize);
}

void serialCallback(SerialMessage &msg) {
  if (strcmp(msg.command, "res/nodes") == 0) {

    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);
    pubSubClient.publish(TOPIC_MESH_NODE_RESPONSE, message.c_str());

    // std::vector<uint32_t> list;
    // for (int i = 0; i < msg.length; i += 4) {
    //   uint32_t val;
    //   memcpy(&val, &msg.payload[i], 4);
    //   list.push_back(val);
    // }
    
    // String joined;
    // for (int i = 0; i < list.size(); ++i) {
    //     joined += String(list[i]);
    //     if (i != list.size() - 1) {
    //         joined += ", ";
    //     }
    // }

    // String out = "{\"nodes\":[" + joined + "]}";

    // pubSubClient.publish(TOPIC_MESH_NODE_RESPONSE, out.c_str());
  }

  else if (strcmp(msg.command, "res/scan") == 0) {

    std::vector<MeshNode> nodes;

    if (msg.length < 4) {
      // Not enough data to even read node count
      return;
    }

    const uint8_t *ptr = msg.payload;

    // Step 1: Read node count (4 bytes)
    uint32_t count = 0;
    memcpy(&count, ptr, 4);
    ptr += 4;

    // Step 2: Read each node
    for (uint32_t i = 0; i < count; i++) {
      if (ptr >= msg.payload + msg.length) break; // Prevent buffer overrun

      // Read MAC address length
      uint8_t macLen = *ptr++;
      if (ptr + macLen > msg.payload + msg.length) break;

      std::string mac((const char*)ptr, macLen);
      ptr += macLen;

      // Read display name length
      if (ptr >= msg.payload + msg.length) break;
      uint8_t nameLen = *ptr++;
      if (ptr + nameLen > msg.payload + msg.length) break;

      std::string name((const char*)ptr, nameLen);
      ptr += nameLen;

      nodes.push_back({mac, name});
    }

    String json = "[";  // Start JSON array

    for (size_t i = 0; i < nodes.size(); i++) {
        const auto& node = nodes[i];
        json += "{";
        json += String("\"mac_address\":\"") + node.mac_address.c_str() + "\",";
        json += String("\"display_name\":\"") + node.display_name.c_str() + "\"";
        json += "}";

        if (i < nodes.size() - 1) {
            json += ", ";  // Comma between objects
        }
    }

    json += "]";

    String output = String("{\"result\":") + json +"}";
    pubSubClient.publish(TOPIC_MESH_SCAN_RESPONSE, output.c_str());

  }

  else if (strcmp(msg.command, "res/pair") == 0) {
    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);
    pubSubClient.publish(TOPIC_MESH_PAIR_RESPONSE, message.c_str());
  }

  else if (strcmp(msg.command, "res/unpair") == 0) {
    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);
    pubSubClient.publish(TOPIC_MESH_UNPAIR_RESPONSE, message.c_str());
  }

  else if (strcmp(msg.command, "res/allnodes") == 0) {
    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);
    pubSubClient.publish(TOPIC_MESH_ALL_NODE_RESPONSE, message.c_str());
  }

  else if (strcmp(msg.command, "mesh/node/info") == 0) {
    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);
    pubSubClient.publish(TOPIC_MESH_PAIR_RESPONSE, message.c_str());
  }

  else if (strcmp(msg.command, "res/state/update") == 0) {
    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);
    pubSubClient.publish(TOPIC_STATE_UPDATE_RESPONSE, message.c_str());
  }

  else if (strcmp(msg.command, "res/mode/update") == 0) {
    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);
    pubSubClient.publish(TOPIC_MODE_UPDATE_RESPONSE, message.c_str());
  }

  else if (strcmp(msg.command, "res/state/get") == 0) {
    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);
    pubSubClient.publish(TOPIC_STATE_GET_RESPONSE, message.c_str());
  }
  else if (strcmp(msg.command, "res/config/get") == 0) {
    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);
    pubSubClient.publish(TOPIC_CONFIGURATION_GET_RESPONSE, message.c_str());
  }
  else if (strcmp(msg.command, "res/config/set") == 0) {
    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);
    pubSubClient.publish(TOPIC_CONFIGURATION_SET_RESPONSE, message.c_str());
  }
  else if (strcmp(msg.command, "res/config/rem") == 0) {
    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);
    pubSubClient.publish(TOPIC_CONFIGURATION_REMOVE_RESPONSE, message.c_str());
  }
  else if (strcmp(msg.command, "auto_update") == 0) {
    std::string message(msg.length, '\0');
    memcpy(message.data(), msg.payload, msg.length);
    pubSubClient.publish(TOPIC_STATE_AUTO_UPDATE, message.c_str());
  }
}