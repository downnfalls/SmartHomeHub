#include "painlessMesh.h"
#include "NimBLEDevice.h"
#include "WiFi.h"
#include "Preferences.h"
#include <ArduinoJson.h>
#include <Wire.h>
#include <map>

#define GPIO0   0
#define GPIO1   1
#define GPIO3   3
#define SDAPIN  8
#define SCLPIN  9

#define TEMPERATURE_SENSOR_ADDRESS  0x5C
#define LIGHT_SENSOR_ADDRESS        0x23

#define GPIO_CHECK_INTERVAL 50
#define I2C_CHECK_INTERVAL 3000

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
std::map<std::string, bool> gpioStateMap;
DynamicJsonDocument gpioMode(256);
uint64_t lastTime = 0;
uint64_t lastTime2 = 0;
bool temperatureSensorConnected = false;
bool lightSensorConnected = false;

bool gpio0 = false;
bool gpio1 = false;
bool gpio3 = false;

float temperature = 0.0;
float humidity = 0.0;
float lux = 0.0;

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

  bool tempSensorExists = deviceExists(TEMPERATURE_SENSOR_ADDRESS);
  bool lightSensorExists = deviceExists(LIGHT_SENSOR_ADDRESS);

  DynamicJsonDocument output(256);
  DynamicJsonDocument gpio(128);
  DynamicJsonDocument i2c(128);

  gpio["gpio_0"] = gpio0;
  gpio["gpio_1"] = gpio1;
  gpio["gpio_3"] = gpio3;

  if (tempSensorExists) i2c["temperature"] = temperature;
  if (tempSensorExists) i2c["humidity"] = humidity;
  if (lightSensorExists) i2c["light"] = lux;

  output["gpio"] = gpio;
  output["i2c"] = i2c;

  String outputStr;
  serializeJson(output, outputStr);

  meshSend(mesh_gateway, "state_update_sensor", outputStr);
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

    else if (cmd == "update_mode") {

      pref.begin("gpio_mode", false);
      pref.putString("gpio_mode", payload);

      DynamicJsonDocument json(256);
      DeserializationError error = deserializeJson(json, payload);
      if (!error) {
        gpioMode = json;
        pref.end();
        meshSend(mesh_gateway, "update_mode_response", "OK");
      }
    }
    else if (cmd == "get_state") {
      sendStateUpdate();
    }

    else if (cmd == "get_mode") {
      String outputStr;
      serializeJson(gpioMode, outputStr);

      meshSend(mesh_gateway, "get_mode_response", outputStr);
    }
  }
}

void meshSend(uint32_t mesh_id, const char* cmd, String text) {

  String output = String(cmd) + '|' + String(text);

  mesh.sendSingle(mesh_id, output.c_str());
}

void sendMeshNodeInfo(uint32_t node_id, bool pair_res) {
  String json = "{\"mac_address\":\""+String(WiFi.macAddress())+"\",\"node_id\": " + String(mesh.getNodeId()) + ",\"node_type\": \"sensor\"}";
  if (pair_res) {
    meshSend(mesh_gateway, "pair_res_info", json.c_str());
  } else {
    meshSend(mesh_gateway, "node_info", json.c_str());
  }
  
}

// =====================================================
//              GPIO INPUT MODE CONFIGURE
// =====================================================

class ButtonModeProcess;
class ToggleModeProcess;
class DelayModeProcess;

std::map<std::string, bool> toggleState;
std::map<std::string, bool> lastState;
std::map<std::string, uint64_t> lastKeep;
class GPIOModeProcess {
  protected:
    bool gp;
    std::string key;
    GPIOModeProcess(bool gp, std::string key) : gp(gp), key(key) {}
  public:
    virtual bool process() = 0;
};
class ButtonModeProcess : public GPIOModeProcess {
  public:
    ButtonModeProcess(bool gp, const std::string& key) : GPIOModeProcess(gp, key) {}
    bool process() override { return gp; }
};
class ToggleModeProcess : public GPIOModeProcess {
  public:
    ToggleModeProcess(bool gp, const std::string& key) : GPIOModeProcess(gp, key) {}
    bool process() override {
      if (gp != lastState[key]) {
        lastState[key] = gp;
        if (gp) {
          toggleState[key] = !toggleState[key];
        }
      }
      return toggleState[key];
    }
};
class DelayModeProcess : public GPIOModeProcess {
  public:
    DelayModeProcess(bool gp, const std::string& key) : GPIOModeProcess(gp, key) {}
    bool process() override {
      if (gp != lastState[key]) {
        lastState[key] = gp;
        if (!gp) {
          lastKeep[key] = millis();
        }
      }

      return gp || millis() - lastKeep[key] <= 10000;
    }
};

bool processGPIO(const String& mode, bool gp, const std::string& key) {
    if (mode == "button") {
        ButtonModeProcess obj(gp, key);
        return obj.process();
    } else if (mode == "toggle") {
        ToggleModeProcess obj(gp, key);
        return obj.process();
    } else if (mode == "delay") {
        DelayModeProcess obj(gp, key);
        return obj.process();
    }
    return false;
}

// =====================================================
//                      I2C LOGIC
// =====================================================
bool deviceExists(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

bool readAM2320() {

  bool result = false;
  // Wake up
  Wire.beginTransmission(TEMPERATURE_SENSOR_ADDRESS);
  Wire.endTransmission();

  // Send read command
  Wire.beginTransmission(TEMPERATURE_SENSOR_ADDRESS);
  Wire.write(0x03);
  Wire.write(0x00);
  Wire.write(0x04);
  Wire.endTransmission();

  // Read 8 bytes
  Wire.requestFrom(TEMPERATURE_SENSOR_ADDRESS, 8);
  if (Wire.available() == 8) {
    uint8_t data[8];
    for (int i = 0; i < 8; i++) data[i] = Wire.read();

    float h = ((data[2] << 8) | data[3]) / 10.0;
    float t = ((data[4] << 8) | data[5]) / 10.0;

    if (humidity != h || temperature != t) {
      result = true;
    }

    humidity = h;
    temperature = t;
  }

  return result;
}

bool readBH1750() {
  bool result = false;
  // Request 2 bytes from the sensor
  Wire.requestFrom(LIGHT_SENSOR_ADDRESS, 2);
  
  if (Wire.available() == 2) {
    uint16_t level = Wire.read() << 8 | Wire.read();
    float l = level / 1.2;

    if (lux != l) {
      result = true;
    }

    lux = l;
  }

  return result;
}

// =====================================================
//               SENSOR TRIGGER BEHAVIER
// =====================================================

void triggerStateUpdate() {

  // GPIO
  if (millis() - lastTime >= GPIO_CHECK_INTERVAL) {
    lastTime = millis();

    bool gp0 = digitalRead(GPIO0);
    bool gp1 = digitalRead(GPIO1);
    bool gp3 = digitalRead(GPIO3);

    gpio0 = processGPIO(gpioMode["gpio_0"], gp0, "gpio_0");
    gpio1 = processGPIO(gpioMode["gpio_1"], gp1, "gpio_1");
    gpio3 = processGPIO(gpioMode["gpio_3"], gp3, "gpio_3");

    bool send = gpioStateMap["gpio0"] != gpio0 || gpioStateMap["gpio1"] != gpio1 || gpioStateMap["gpio3"] != gpio3;

    gpioStateMap["gpio0"] = gpio0;
    gpioStateMap["gpio1"] = gpio1;
    gpioStateMap["gpio3"] = gpio3;

    if (send) {
      sendStateUpdate();
    }
  }

  // I2C
  if (millis() - lastTime2 >= I2C_CHECK_INTERVAL) {
    lastTime2 = millis();

    bool send1 = false;
    bool send2 = false;
    bool send3 = false;

    bool tempSensorExists = deviceExists(TEMPERATURE_SENSOR_ADDRESS);
    bool lightSensorExists = deviceExists(LIGHT_SENSOR_ADDRESS);

    if (tempSensorExists) {
      send1 = readAM2320();
      temperatureSensorConnected = true;
    } else {
      if (temperatureSensorConnected) {
        send3 = true;
      }
      temperatureSensorConnected = false;
    }

    if (lightSensorExists) {

      if (!lightSensorConnected) {
        Wire.beginTransmission(LIGHT_SENSOR_ADDRESS);
        Wire.write(0x10);       // Continuous high-res mode
        Wire.endTransmission();
      }

      lightSensorConnected = true;
      send2 = readBH1750();
    } else {
      if (lightSensorConnected) {
        send3 = true;
      }
      lightSensorConnected = false;
    }

    if (send1 || send2 || send3) {
      sendStateUpdate();
    }
  }
}

// =====================================================
//                  SETUP FUNCTION
// =====================================================

void setup() {
  Serial.begin(115200);

  pinMode(GPIO0, INPUT_PULLDOWN);
  pinMode(GPIO1, INPUT_PULLDOWN);
  pinMode(GPIO3, INPUT_PULLDOWN);
  pinMode(2, OUTPUT);
  digitalWrite(BUILTIN_LED, HIGH);

  // set gpio mode
  pref.begin("gpio_mode", false);

  if (pref.getString("gpio_mode").isEmpty()) {

    DynamicJsonDocument mode(256);
    mode["gpio_0"] = "button";
    mode["gpio_1"] = "button";
    mode["gpio_3"] = "button";

    String modeConfig;
    serializeJson(mode, modeConfig);
    pref.putString("gpio_mode", modeConfig);

    gpioMode = mode;
  } else {
    String modeConfig = pref.getString("gpio_mode");
    DynamicJsonDocument mode(256);
    DeserializationError error = deserializeJson(mode, modeConfig);
    if (!error) {
      gpioMode = mode;
    }
  }

  pref.end();

  String modeStr;
  serializeJson(gpioMode, modeStr);
  Serial.println(String("Mode HERE: ") + modeStr);

  esp_log_level_set("wifi", ESP_LOG_NONE);

  WiFi.mode(WIFI_AP_STA);

  // I/O
  Wire.begin(SDAPIN, SCLPIN);

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
    Serial.println("No mesh credentials found.");
    Serial.println("Start bluetooth advertising...");

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
    Serial.println("aaa");
    mesh.update();
 
    if (mesh.isConnected(mesh_gateway) && !info) {
      info = true;
      sendMeshNodeInfo(mesh.getNodeId(), first_init);
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

      pref.begin("gpio_mode", false);
      pref.remove("gpio_mode");
      pref.end();

      Serial.println("Disconnected from mesh network.\nRebooting...");
      ESP.restart();
    }
    
    
  }

  // Sensor Detect Logic
  triggerStateUpdate();
}

