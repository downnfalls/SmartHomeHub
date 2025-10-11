## 🏡 Smart Mesh Network System

This project is a set of software for **ESP32** devices designed to create a **Mesh Network** for a smart home system. It utilizes the **painlessMesh** library for communication and **NimBLE** (Bluetooth Low Energy) for initial device setup (Provisioning).

The system consists of three main device roles:

1.  **Gateway Node (Root):** The central communication and automation manager.
2.  **Sensor Node (Input):** A device that detects and reports status (GPIOs, I2C Sensors).
3.  **Relay Node (Output):** A device that controls switches/relays based on commands.

---

## 💻 Code Structure (Code Overview)

| File | Role (Node Type) | Brief Description |
| :--- | :--- | :--- |
| `gateway_node.ino` | **Gateway (Root)** | Manages the Mesh network, acts as a **BLE Scanner** for new device pairing, handles **Automation Logic**, and communicates with a **Host Controller** via Serial2. |
| `sensor_node.ino` | **Sensor (Input)** | Monitors **GPIO** status (Button, Toggle, Delay modes) and **I2C Sensors** (e.g., AM2320/BH1750), and reports status changes back to the Gateway. |
| `relay_node.ino` | **Relay (Output)** | Receives commands from the Gateway to control **Relay Outputs** (R1/R2) and reports the current state of the relays back to the Gateway. |

---

## 🛠️ Setup and Usage

### 1. Prerequisites

* **Board:** ESP32 (supports Wi-Fi and BLE)
* **Software:** Arduino IDE or PlatformIO
* **Required Libraries:**
    * `painlessMesh`
    * `NimBLE-Arduino`
    * `ArduinoJson`
    * `Preferences`
    * `Wire`

### 2. Gateway Node

* **Connection:** This node joins the Mesh network using hardcoded credentials:
    * `#define MESH_PREFIX "MeshHome"`
    * `#define MESH_PASSWORD "2147483647"`
    * `#define MESH_PORT 5555`
* **Key Functions:**
    * Acts as the **Root Node** of the network.
    * Scans for new Sensor/Relay nodes advertising via **BLE** (pairing mode).
    * Routes status data and commands between nodes.
    * Manages **Automation** rules (e.g., when Sensor A is active, turn Relay B off).
    * Communicates with an external system (Host Controller) via **Serial2** (TX=17, RX=16) using a custom protocol.

### 3. Sensor and Relay Nodes

These nodes operate in two modes:

| Mode | Condition | Behavior |
| :--- | :--- | :--- |
| **1. Configuration Mode (BLE)** | When no Mesh credentials (SSID/Password/Gateway ID) are found in NVS. | Advertises via **BLE** to wait for network configuration data from the Gateway or an external app. |
| **2. Operational Mode (Mesh)** | When Mesh credentials are saved in NVS. | Automatically connects to the Mesh network and begins reporting status or awaiting commands. |

#### Pairing Process

1.  Upload the code to the Sensor or Relay node.
2.  The node boots into **Configuration Mode** (BLE Advertising).
3.  The Gateway Node (or an application) performs a **BLE Scan** and discovers the new node.
4.  The Gateway sends the Mesh Credentials (Gateway ID, Mesh SSID, Password, Port) via BLE to the node.
5.  The node saves the data to NVS, stops BLE, and reboots into **Operational Mode** to join the Mesh network.

#### Specific Node Features

* **Sensor Node:** The mode for GPIO inputs can be configured (`button`, `toggle`, or `delay`) via an `update_mode` command from the Gateway.
* **Relay Node:** Receives the `update_state` command, which contains a JSON payload (`{"r1":true, "r2":false}`) to control the physical relays.

---

## 🤝 Communication Protocol

### A. Mesh Protocol (Command\|Payload)

| Node Type | Command | Example Payload | Description |
| :--- | :--- | :--- | :--- |
| **Relay -> Gateway** | `state_update_relay` | `{"r1":true,"r2":false}` | Reports the current relay state. |
| **Sensor -> Gateway** | `state_update_sensor` | `{"gpio":{"gpio_0":true},"i2c":{"temperature":25.5}}` | Reports the state of inputs and sensors. |
| **Gateway -> Node** | `update_state` | `{"r1":false,"r2":true}` | Commands a Relay Node to change its state. |
| **Gateway -> Node** | `unpair` | `unpair` | Commands the node to erase Mesh data and reboot. |
| **Node -> Gateway** | `pair_res_info` | `{"mac_address":"...", "node_type":"relay"}` | Reports node information after pairing. |

### B. Serial Protocol (Gateway <-> Host Controller)

The Gateway uses UART2 (Serial2) to communicate with an external host (e.g., Raspberry Pi or another MCU) using a binary protocol defined by a Start Byte (`0xAA`), Command (16 Bytes), Length, Payload, and Checksum for reliable transmission.

* **Example:** Commands the Gateway forwards to the Host: `state_update_relay`
* **Example:** Host sends a command to the Gateway via Serial: `MESH_SEND` (to forward a command to a specific Mesh node).
