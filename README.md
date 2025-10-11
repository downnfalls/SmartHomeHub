Of course\! Here is a complete `README.md` file for your Smart Home IoT project based on the code you provided.

-----

# Smart Home IoT Project 🏠✨

A modular and scalable Smart Home system built using an ESP32-based Wi-Fi mesh network, managed by a central gateway that communicates with the outside world via an MQTT bridge. This project allows for easy integration of various sensor and actuator nodes, dynamic pairing via Bluetooth, and a powerful automation engine.

## Table of Contents

  - [System Architecture](https://www.google.com/search?q=%23system-architecture)
  - [Key Features](https://www.google.com/search?q=%23key-features-%F0%9F%9A%80)
  - [Hardware Required](https://www.google.com/search?q=%23hardware-required-%F0%9F%94%8C)
  - [Software & Libraries](https://www.google.com/search?q=%23software--libraries-%F0%9F%92%BB)
  - [Setup and Installation](https://www.google.com/search?q=%23setup-and-installation-%E2%9A%99%EF%B8%8F)
  - [Communication Protocols](https://www.google.com/search?q=%23communication-protocols-%F0%9F%93%A1)
  - [How It Works](https://www.google.com/search?q=%23how-it-works-%F0%9F%9B%A0%EF%B8%8F)
  - [Automation Engine](https://www.google.com/search?q=%23automation-engine-%F0%9F%A7%A0)

-----

## System Architecture

The system is composed of four main firmware components running on separate microcontrollers, creating a robust and decoupled architecture.

1.  **MasterMesh (ESP32 Gateway)**: The brain of the system. It acts as the root node of a `painlessMesh` network, managing all child nodes. It handles BLE pairing for new devices, stores configurations, and runs the automation logic. It communicates with the MasterWifi bridge via a UART serial connection.

2.  **MasterWifi (ESP8266 Bridge)**: The communication link to the outside world. It connects to your local Wi-Fi and an MQTT broker. It acts as a transparent bridge, translating MQTT commands into serial commands for the MasterMesh and forwarding responses and updates from the mesh back to the MQTT broker.

3.  **ChildRelay (ESP32 Node)**: An actuator node. It joins the mesh network and controls two relays based on commands received from the MasterMesh gateway.

4.  **ChildSensor (ESP32 Node)**: A sensor node. It reads data from GPIO inputs and I2C sensors (temperature, humidity, light) and reports any state changes to the MasterMesh gateway.

### Data Flow Diagram

```
 [ MQTT Broker ] <-----> [ Wi-Fi ] <-----> [ MasterWifi ESP8266 ] <-----> [ UART ] <-----> [ MasterMesh ESP32 ]
                                                                                                  ^
                                                                                                  |
                                                                                            [painlessMesh]
                                                                                                  |
                                                               <----------------------------------+---------------------------------->
                                                               |                                                                    |
                                                         [ ChildRelay ESP32 ]                                             [ ChildSensor ESP32 ]
```

-----

## Key Features 🚀

  - **Modular Mesh Network**: Utilizes `painlessMesh` for reliable and self-healing communication between nodes, eliminating the need for every device to be in range of a central router.
  - **Centralized Gateway**: A single Master node manages all child nodes, states, and automation rules, simplifying control and logic.
  - **MQTT Integration**: Connects the local mesh to any standard MQTT broker, enabling remote control, monitoring, and integration with other platforms like Home Assistant or Node-RED.
  - **Simple BLE Pairing**: New nodes are discovered and configured using Bluetooth Low Energy. Just power on a new node and pair it from your control application—no hardcoding of credentials needed.
  - **Persistent Storage**: Node information and automation configurations are saved in the ESP32's non-volatile storage, ensuring they persist after a reboot.
  - **Powerful Automation**: Create rules on the gateway to trigger actions on relay nodes based on sensor states (e.g., "turn on the fan if the temperature is above 25°C").
  - **Configurable Sensor Modes**: GPIO inputs on sensor nodes can be configured to act as a direct button, a toggle switch, or a motion sensor with a delay.

-----

## Hardware Required 🔌

| Component         | Microcontroller                    | Peripherals                                                |
| ----------------- | ---------------------------------- | ---------------------------------------------------------- |
| **MasterMesh** | ESP32 Development Board            | -                                                          |
| **MasterWifi** | ESP8266 Board (NodeMCU, Wemos D1)  | -                                                          |
| **ChildRelay** | ESP32 Development Board            | 2-Channel Relay Module                                     |
| **ChildSensor** | ESP32 Development Board            | AM2320 (Temp/Humidity), BH1750 (Light), Buttons/Switches   |

-----

## Software & Libraries 💻

Ensure you have the Arduino IDE or PlatformIO set up with the ESP32 and ESP8266 board managers. Install the following libraries through the Library Manager.

  - **Common (for ESP32)**:

      - `painlessMesh`
      - `NimBLEDevice` (for BLE communication)
      - `ArduinoJson` (for data serialization)
      - `Preferences` (built-in)
      - `Wire` (built-in for I2C)

  - **MasterWifi (for ESP8266)**:

      - `PubSubClient` (for MQTT)
      - `WiFiManager` (for easy Wi-Fi configuration)
      - `SoftwareSerial` (built-in)
      - `EEPROM` (built-in)

-----

## Setup and Installation ⚙️

1.  **Configuration**:

      - In `MasterMesh.ino`, you can customize the mesh network credentials:
        ```cpp
        #define MESH_PREFIX     "MeshHome"
        #define MESH_PASSWORD   "2147483647"
        #define MESH_PORT       5555
        ```

2.  **Wiring**:

      - Connect the **MasterMesh (ESP32)** and **MasterWifi (ESP8266)** boards. **A logic level shifter is recommended if you are not using 5V tolerant pins on the ESP32.**
          - `MasterMesh GPIO 17 (TX)` -\> `MasterWifi GPIO 13 (RX)`
          - `MasterMesh GPIO 16 (RX)` -\> `MasterWifi GPIO 15 (TX)`
          - `GND` -\> `GND`
      - Connect peripherals (relays, sensors) to the child nodes according to the pin definitions in their respective code files.

3.  **Flashing**:

      - Flash `MasterMesh.ino` to the ESP32 Gateway.
      - Flash `MasterWifi.ino` to the ESP8266 Bridge.
      - Flash `ChildRelay.ino` and `ChildSensor.ino` to their respective ESP32 boards.

4.  **First-Time Wi-Fi Setup**:

      - On its first boot, the **MasterWifi** board will create a Wi-Fi Access Point named `SmartHub-Setup`.
      - Connect to this network with your phone or computer. A captive portal will open.
      - Enter your home Wi-Fi credentials and the IP address of your MQTT broker.
      - The device will save the configuration and connect to your network.

-----

## Communication Protocols 📡

The system uses three distinct communication protocols for different parts of the architecture.

### 1\. UART (MasterMesh \<-\> MasterWifi)

A custom binary protocol ensures reliable serial communication.

  - **Format**: `[START_BYTE (0x00)] [16-BYTE COMMAND] [PAYLOAD] [END_BYTE (0xFF)]`
  - This format allows for well-defined commands and variable-length data payloads.

### 2\. MQTT Topics (MasterWifi \<-\> Broker)

The system uses a simple `request/response` pattern for most interactions. You publish a command to a `req/...` topic and listen for results on the corresponding `res/...` topic.

**Key Topics:**

  - `req/mesh/scan`: Start a BLE scan for new nodes.
  - `res/mesh/scan`: Receives a JSON array of discovered nodes.
  - `req/mesh/pair`: Send a MAC address to pair a new node.
  - `res/mesh/pair`: Receives pairing status and new node info.
  - `req/mesh/unpair`: Send a `node_id` to remove a node from the network.
  - `req/state/update`: Send a JSON object to update the state of a node (e.g., turn on a relay).
  - `req/config/set`: Send a JSON object to define an automation rule.
  - `res/state/auto_update`: Listen to this topic for state changes triggered by automations.

### 3\. Mesh Messages (MasterMesh \<-\> Child Nodes)

Internal communication within the mesh network uses simple string messages delimited by a pipe `|`.

  - **Format**: `command|payload`
  - **Examples**:
      - `unpair|`: Command from gateway to tell a node to forget the network.
      - `update_state|{"r1":true,"r2":false}`: Command from gateway to update a relay's state.
      - `state_update_sensor|{"gpio":{"gpio_0":true},"i2c":{...}}`: A sensor node reporting its new state to the gateway.

-----

## How It Works 🛠️

1.  **Boot-Up**: The Gateway and Bridge start up. Child nodes without saved credentials enter BLE advertising mode.
2.  **Pairing a New Node**:
      - Publish an empty message to `req/mesh/scan` via your MQTT client.
      - The Gateway scans for BLE devices. A list of found nodes is published to `res/mesh/scan`.
        ```json
        {"result":[{"mac_address":"AA:BB:CC:11:22:33","display_name":"MeshNode"}]}
        ```
      - Publish the `mac_address` of the desired node to `req/mesh/pair`.
      - The Gateway connects to the node via BLE, sends it the mesh credentials, and the node reboots to join the network.
      - A success message with the new node's info is published to `res/mesh/pair`.
        ```json
        {"mac_address":"AA:BB:CC:11:22:33","node_id":123456789,"node_type":"relay","info":"pair success!"}
        ```
3.  **Controlling a Relay**:
      - To turn on relay 1 (`r1`) of a node, publish a JSON payload to `req/state/update`.
      - Payload: `{"node_id": 123456789, "state": {"r1": true}}`
4.  **Monitoring Nodes**:
      - Sensor nodes automatically report any changes to the Gateway.
      - To get a list of all *currently online* nodes and their cached states, publish to `req/mesh/nodes`. The response will be sent to `res/mesh/nodes`.

-----

## Automation Engine 🧠

The automation logic runs exclusively on the **MasterMesh Gateway**, providing a central point for all rules.

### How to Create a Rule

You can set or update rules by publishing a JSON object to the `req/config/set` MQTT topic.

  - **Structure**: The root of the JSON object contains keys that are the `node_id` of the **relay node** you want to control.
  - **Condition**: Inside each relay node object, you define conditions for `r1` and `r2`. Each condition specifies the `node_id` of the triggering **sensor node** and a `condition` string.

### Example

Let's say you have:

  - A **Relay Node** with `node_id`: `33445566`
  - A **Sensor Node** with `node_id`: `11223344` that has a temperature sensor and a button on `gpio_0`.

To turn on **Relay 1** when the temperature exceeds 25.5°C and turn on **Relay 2** when the button is pressed, you would publish the following JSON to `req/config/set`:

```json
{
  "33445566": {
    "r1": {
      "node_id": 11223344,
      "condition": "temperature>25.5"
    },
    "r2": {
      "node_id": 11223344,
      "condition": "gpio_0|true"
    }
  }
}
```

### Condition String Format

The `condition` string is parsed by the gateway and supports the following formats:

  - For I2C sensors (like temperature, light): `sensor_name>value`, `sensor_name<value`, `sensor_name>=value`, etc.
  - For GPIO sensors: `gpio_name|state` (e.g., `gpio_0|true` or `gpio_1|false`).

When a sensor node reports a state change, the gateway evaluates all relevant rules. If a condition is met, it automatically sends an `update_state` command to the target relay node and also publishes this change to the `res/state/auto_update` MQTT topic.
