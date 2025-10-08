#include "painlessMesh.h"
#include "NimBLEDevice.h"
#include "WiFi.h"
#include "Preferences.h"
#include <ArduinoJson.h>

#define R1PIN 6
#define R2PIN 7

void meshCallback(uint32_t from, String &msg);
void meshSend(uint32_t mesh_id, const char *cmd, String text);
void bluetoothAdvertise();

painlessMesh mesh;
Preferences pref;
static NimBLEServer* pServer;

bool mesh_init = false;
uint32_t mesh_gateway = 0;
String mesh_ssid = "";
String mesh_password = "";
uint32_t mesh_port = 0;
bool first_init = false;

bool r1 = false;
bool r2 = false;

// =====================================================
//                   BLE LOGIC
// =====================================================

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    Serial.printf("Client address: %s\n", connInfo.getAddress().toString().c_str());

    NimBLEDevice::getAdvertising()->stop();
  }
};

class GatewayCharacteristicCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    std::string value = pCharacteristic->getValue();
    Serial.print("Received Gateway: ");
    Serial.println(value.c_str());

    uint32_t cast = static_cast<uint32_t>(std::stoul(value));
    pref.begin("mesh", false);
    pref.putUInt("mesh_gateway", cast);
    uint32_t debug = pref.getUInt("mesh_gateway", 0);
    Serial.println(String(debug));
    pref.end();
    mesh_gateway = cast;
  }
};

class MeshSSIDCharacteristicCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    std::string value = pCharacteristic->getValue();
    Serial.print("Received SSID: ");
    Serial.println(value.c_str());

    pref.begin("mesh", false);
    pref.putString("mesh_ssid", String(value.c_str()));
    pref.end();
    mesh_ssid = String(value.c_str());
  }
};

class MeshPasswordCharacteristicCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    std::string value = pCharacteristic->getValue();
    Serial.print("Received Password: ");
    Serial.println(value.c_str());

    pref.begin("mesh", false);
    pref.putString("mesh_password", String(value.c_str()));
    pref.end();
    mesh_password = String(value.c_str());
  }
};

class MeshPortCharacteristicCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    std::string value = pCharacteristic->getValue();
    Serial.print("Received Port: ");
    Serial.println(value.c_str());

    uint16_t cast = static_cast<uint16_t>(std::stoul(value));
    pref.begin("mesh", false);
    pref.putUInt("mesh_port", cast);
    pref.end();
    mesh_port = cast;

    if (!mesh_ssid.isEmpty() && !mesh_password.isEmpty() && mesh_gateway != 0) {
      WiFi.disconnect(true);
      delay(100);  
      Serial.println("Initializing mesh network...");
      mesh.init(mesh_ssid, mesh_password, mesh_port);
      mesh_init = true;
      first_init = true;
    }
  }
};

void bluetoothAdvertise() {
  NimBLEDevice::init("MeshNode");
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService *pService = pServer->createService("c220cb58-0d9f-4405-a55a-da4794291e8f");

  NimBLECharacteristic* nodeTypeCharacteristic = pService->createCharacteristic("dc38921e-e997-46c4-9ff6-3cb91c6d15d5", NIMBLE_PROPERTY::READ);
  NimBLECharacteristic* gatewayCharacteristic = pService->createCharacteristic("46f42176-8d03-4241-843d-195aea0ea390", NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  NimBLECharacteristic* meshSSIDCharacteristic = pService->createCharacteristic("98da85e8-42d6-493f-8bff-5416b686f1d2", NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  NimBLECharacteristic* meshPasswordCharacteristic = pService->createCharacteristic("207cd133-188a-4503-8a94-17c6070aadce", NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  NimBLECharacteristic* meshPortCharacteristic = pService->createCharacteristic("fac0fc9c-f966-4685-bd92-ed4332481bf4", NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);

  gatewayCharacteristic->setCallbacks(new GatewayCharacteristicCallback());
  meshSSIDCharacteristic->setCallbacks(new MeshSSIDCharacteristicCallback());
  meshPasswordCharacteristic->setCallbacks(new MeshPasswordCharacteristicCallback());
  meshPortCharacteristic->setCallbacks(new MeshPortCharacteristicCallback());

  pService->start();
  nodeTypeCharacteristic->setValue("OUTPUT");

  NimBLEAdvertisementData advData;

  String mac = WiFi.macAddress();
  advData.setManufacturerData(mac.c_str());

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setAdvertisementData(advData);
  pAdvertising->setName("MeshNode");
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->enableScanResponse(true);
  pAdvertising->start();
}

// =====================================================
//                     MESH LOGIC
// =====================================================

void sendStateUpdate() {
  meshSend(mesh_gateway, "state_update", String("{\"r1\":")+String(r1 ? "true" : "false")+",\"r2\":"+String(r2 ? "true" : "false")+"}");
}

bool unpairing = false;

void meshCallback(uint32_t from, String &msg) {

  int sep = msg.indexOf('|');

  String cmd = msg.substring(0, sep);      // before '|'
  String payload = msg.substring(sep + 1);

  Serial.println("mesh callback receive!");

  if (from == mesh_gateway) {
    if (cmd == "unpair") {

      meshSend(mesh_gateway, "unpairing", String(WiFi.macAddress()));

      unpairing = true;

    }

    else if (cmd == "update_state" || cmd == "state_response") {

      DynamicJsonDocument state(512);
      DeserializationError error = deserializeJson(state, payload);

      if (!error) {
        if (state.containsKey("r1")) r1 = state["r1"];
        if (state.containsKey("r2")) r2 = state["r2"];

        Serial.println(String("R1: ") + ((r1) ? "true" : "false") + "\nR2: " + ((r2) ? "true" : "false"));

        digitalWrite(R1PIN, r1);
        digitalWrite(R2PIN, r2);
      }

      if (cmd == "update_state") meshSend(mesh_gateway, "update_state_response", "OK");
    }

    else if (cmd == "get_state") {
      sendStateUpdate();
    }
  }
}

void meshSend(uint32_t mesh_id, const char* cmd, String text) {

  String output = String(cmd) + '|' + String(text);

  mesh.sendSingle(mesh_id, output.c_str());
  
}

void sendMeshNodeInfo(uint32_t node_id, bool pair_res) {
  String json = "{\"mac_address\":\""+String(WiFi.macAddress())+"\",\"node_id\": " + String(mesh.getNodeId()) + ",\"node_type\": \"relay\"}";
  if (pair_res) {
    meshSend(mesh_gateway, "pair_res_info", json.c_str());
  } else {
    meshSend(mesh_gateway, "node_info", json.c_str());
  }
}

void requestForState() {
  meshSend(mesh_gateway, "state_request", "request");
}

// =====================================================
//                  SETUP FUNCTION
// =====================================================

void setup() {
  Serial.begin(115200);

  esp_log_level_set("wifi", ESP_LOG_NONE);

  WiFi.mode(WIFI_AP_STA);

  pinMode(R1PIN, OUTPUT);
  pinMode(R2PIN, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(BUILTIN_LED, HIGH);

  pref.begin("mesh", true);
  // mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
  mesh.onReceive(&meshCallback);
  mesh.setContainsRoot(true);

  mesh_gateway = pref.getUInt("mesh_gateway", 0);
  mesh_ssid = pref.getString("mesh_ssid", "");
  mesh_password = pref.getString("mesh_password", "");
  mesh_port = pref.getUInt("mesh_port", 0);

  pref.end();

  if (mesh_gateway != 0 && !mesh_ssid.isEmpty() && !mesh_password.isEmpty() && mesh_port != 0) {

    // Connect to mesh network if crecential exists

    Serial.println("Initializing mesh network...");

    Serial.print("mesh_ssid: ");
    Serial.println(mesh_ssid.c_str());
    Serial.print("mesh_password: ");
    Serial.println(mesh_password.c_str());
    Serial.print("mesh_port: ");
    Serial.println(String(mesh_port).c_str());

    mesh.init(mesh_ssid, mesh_password, mesh_port);
    mesh_init = true;


  } else {

    // Start Bluetooth if no mesh credentials
    bluetoothAdvertise();
  }
}

// =====================================================
//                  LOOP FUNCTION
// =====================================================

long long start_unpairing = 0;
bool info = false;
void loop() {

  if (mesh_init) {
    mesh.update();

    if (mesh.isConnected(mesh_gateway) && !info) {
      info = true;
      sendMeshNodeInfo(mesh.getNodeId(), first_init);
      requestForState();
    }

    if (unpairing) {
      unpairing = false;
      start_unpairing = millis();
    }

    if (start_unpairing != 0 && millis() - start_unpairing > 5000) {
      start_unpairing = 0;

      mesh_init = false;
      mesh_gateway = 0;
      mesh_ssid = "";
      mesh_password = "";
      mesh_port = 0;
      mesh.stop();

      pref.begin("mesh", false);
      pref.remove("mesh_gateway");
      pref.remove("mesh_ssid");
      pref.remove("mesh_password");
      pref.remove("mesh_port");
      pref.end();

      Serial.println("Disconnected from mesh network.\nRebooting...");
      ESP.restart();
    }
  }
}

