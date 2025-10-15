<template>
  <div style="display: flex; flex-direction: column" class="ma-3">
    <div class="top-buttons d-lg-flex d-sm-block justify-space-between align-center">
      <div>
        <strong class="text-h6 font-weight-bold text-uppercase text-blue-grey-darken-4 mr-4">
          status :
          <span :class="master_status === 'online' ? 'text-teal' : 'text-red-darken-2'">
            {{ master_status }}
          </span>
        </strong>
      </div>
      <div>
        <v-btn
          outlined
          @click="refreshPage"
          style="background-color: #a2af9b; color: black"
          class="font-weight-bold text-subtitle-1 text-blue-grey-darken-4 mr-4"
        >
          REFRESH
        </v-btn>
        <v-btn
          outlined
          style="background-color: #a2af9b; color: black"
          class="font-weight-bold text-subtitle-1 text-blue-grey-darken-4"
          :loading="isLoading"
          :disabled="isLoading"
          @click="syncSlave"
        >
          <template v-slot:loader>
            <v-progress-circular indeterminate color="white" size="20"></v-progress-circular>
          </template>
          <span v-if="!isLoading">SYNC</span>
          <span v-else>Loading...</span>
        </v-btn>
      </div>
    </div>
    <v-row class="mt-5 pa-4 rounded-lg" style="background-color: #a2af9b; color: black">
      <v-col
        v-for="(node, index) in slave_nodes"
        :key="index"
        cols="12"
        sm="6"
        md="3"
      >
        <v-sheet
          class="pa-3 text-subtitle-1 text-blue-grey-darken-4 rounded-lg"
          elevation="1"
          color="primary"
        >
          <v-sheet color="primary">
            <div class="d-flex justify-space-between align-center">
              <strong>STATUS :
                <span :class="node.node_status === 'online' ? 'text-teal text-uppercase' : 'text-red-darken-2 text-uppercase'">
                  {{ node.node_status }}
                </span>
              </strong>
              <div>
                <v-btn
                  icon="mdi-cog"
                  size="small"
                  variant="text"
                  @click.stop="showNodePopup(node)"
                  :disabled="node.node_status === 'offline'"
                ></v-btn>
                <v-btn
                  icon="mdi-delete"
                  size="small"
                  variant="text"
                  color="red-darken-2"
                  @click.stop="unpairNode(node)"
                  :disabled="node.node_status === 'offline'"
                ></v-btn>
              </div>
            </div>
            <div class="d-flex align-center">
              <span class="mr-2">Node Name :</span>
              <span
                v-if="editingNodeMac !== node.mac_address"
                @click.stop="startEditing(node)"
                style="cursor: pointer;"
                class="font-weight-bold"
              >
                {{ node.name || 'Mesh Node' }}
              </span>
              <v-text-field
                v-else
                v-model="editingNodeName"
                @keyup.enter="saveNodeName"
                @blur="saveNodeName"
                @click.stop
                dense
                autofocus
                single-line
                hide-details
              ></v-text-field>
            </div>
            <p>MAC Addr : {{ node.mac_address }}</p>
            <p>Node ID : {{ node.node_id }}</p>
            <p>Node Type : {{ node.node_type }}</p>
            <p>Description : {{ node.description }}</p>
            
            <div v-if="node.node_type === 'sensor' && node.state && node.state.i2c && node.node_status == 'online'" class="mt-2">
              <v-divider class="my-1"></v-divider>
              <p v-for="(value, key) in node.state.i2c" :key="key" class="mb-0 text-caption">
                <strong class="text-capitalize font-weight-bold">{{ key }}:</strong> {{ value }}
              </p>
              <v-divider class="mt-1"></v-divider>
            </div>
            <div v-if="node.node_type === 'relay' && node.node_status === 'online'">
              
              <v-btn
                class="mb-2 mt-2"
                block
                color="blue-grey-darken-1"
                @click="setManualMode(node)"
              >
                {{ node.manual_mode ? 'AUTO' : 'MANUAL' }}
              </v-btn>
              <v-divider class="my-2"></v-divider>
              <div class="d-flex justify-space-around">
                <v-switch
                  v-model="node.state.r1"
                  label="Relay 1"
                  color="teal"
                  inset
                  hide-details
                  :disabled="!node.manual_mode"
                  @change.stop="toggleRelay(node, 'r1')"
                ></v-switch>
                <v-switch
                  v-model="node.state.r2"
                  label="Relay 2"
                  color="teal"
                  inset
                  hide-details
                  :disabled="!node.manual_mode"
                  @change.stop="toggleRelay(node, 'r2')"
                ></v-switch>
              </div>
            </div>
          </v-sheet>
        </v-sheet>
      </v-col>
    </v-row>
    <v-dialog v-model="dialog" max-width="600">
      <v-card v-if="selectedNode">
        <div class="text-h6 text-center text-blue-grey-darken-4 pa-3" style="background-color: #a2af9b;">
          {{ selectedNode.name || 'Mesh Node' }}
        </div>
        
        <div v-if="selectedNode.node_type === 'sensor' && loadingNodeId === selectedNode.node_id" class="d-flex justify-center align-center pa-10" style="min-height: 150px;">
          <v-progress-circular indeterminate color="primary"></v-progress-circular>
        </div>
        
        <v-list v-else-if="selectedNode.node_type === 'sensor' && editableNodeConfig.state && editableNodeConfig.state.gpio">
          <v-list-item 
            v-for="(gpioConfig, gpioPin) in editableNodeConfig.state.gpio" 
            :key="gpioPin"
          >
            <div class="d-flex flex-column flex-sm-row justify-space-between align-center">
              <div class="font-weight-bold mb-2 mb-sm-0">
                {{ gpioPin }}
              </div>
              <v-btn-toggle
                v-model="gpio_config.mode[gpioPin]"
                variant="outlined"
                divided
                density="compact"
              >
                <v-btn 
                  value="button" 
                  @click="updateGpioConfig(gpioPin, 'button')" 
                >Button</v-btn>
                <v-btn 
                  value="toggle" 
                  @click="updateGpioConfig(gpioPin, 'toggle')" 
                >Toggle</v-btn>
                <v-btn 
                  value="delay" 
                  @click="updateGpioConfig(gpioPin, 'delay')" 
                >Delay</v-btn>
              </v-btn-toggle>
            </div>
          </v-list-item>
        </v-list>
        <v-list v-else-if="selectedNode.node_type === 'relay'">
          <v-list-item>
            <div class="d-flex flex-column flex-sm-row flex-wrap align-start align-sm-center">
              <strong class="mr-sm-4 mb-2 mb-sm-0">Relay 1:</strong>
              <div style="width: 100%;" class="d-flex flex-column flex-sm-row flex-wrap align-start align-sm-center">
                <v-select
                  v-model="editableNodeConfig.state.r1_sensor"
                  :items="sensorNodeList"
                  label="Sensor Node"
                  dense outlined hide-details
                  class="mr-sm-2 mb-2 mb-sm-0 w-100 w-sm-auto"
                  style="max-width: 150px;"
                ></v-select>
                <v-select
                  v-model="editableNodeConfig.state.r1_sensor_type"
                  :items="['gpio', 'i2c']"
                  label="Type"
                  dense outlined hide-details
                  class="mr-sm-2 mb-2 mb-sm-0 w-100 w-sm-auto"
                  style="max-width: 100px;"
                ></v-select>
              </div>
              <div class="d-flex flex-column" style="width: 100%;">
                <span class="text-caption mb-1" :class="{ 'text-disabled': !editableNodeConfig.state.r1_sensor_type }">Pin/Addr:</span>
                <v-btn-toggle
                  v-model="editableNodeConfig.state.r1_sensor_pin"
                  variant="outlined"
                  density="compact"
                  divided
                  :disabled="!editableNodeConfig.state.r1_sensor_type"
                  class="flex-wrap"
                >
                  <v-btn
                    v-for="pin in r1_sensor_pins"
                    :key="pin"
                    :value="pin"
                    class="flex-grow-1"
                  >
                    {{ pin }}
                  </v-btn>
                </v-btn-toggle>
              </div>
              <div class="d-flex flex-column" style="width: 100%;">
                <v-text-field
                  v-if="editableNodeConfig.state.r1_sensor && editableNodeConfig.state.r1_sensor_type && editableNodeConfig.state.r1_sensor_type == 'i2c' && editableNodeConfig.state.r1_sensor_pin"
                  v-model="editableNodeConfig.state.r1_condition_name"
                  label="Work Condition Ex: >30"
                  dense
                  outlined
                  hide-details
                  class="mt-4"
                ></v-text-field>
                <v-btn-toggle
                  v-if="editableNodeConfig.state.r1_sensor && editableNodeConfig.state.r1_sensor_type && editableNodeConfig.state.r1_sensor_type == 'gpio' && editableNodeConfig.state.r1_sensor_pin"
                  v-model="editableNodeConfig.state.r1_condition_name"
                  variant="outlined"
                  density="compact"
                  divided
                  class="mt-4"
                >
                  <v-btn :value="true">TRUE</v-btn>
                  <v-btn :value="false">FAlSE</v-btn>
                </v-btn-toggle>
              </div>
            </div>
          </v-list-item>
          <v-divider></v-divider>
          <v-list-item>
            <div class="d-flex flex-column flex-sm-row flex-wrap align-start align-sm-center">
              <strong class="mr-sm-4 mb-2 mb-sm-0">Relay 2:</strong>
              <div style="width: 100%;" class="d-flex flex-column flex-sm-row flex-wrap align-start align-sm-center">
                <v-select
                  v-model="editableNodeConfig.state.r2_sensor"
                  :items="sensorNodeList"
                  label="Sensor Node"
                  dense outlined hide-details
                  class="mr-sm-2 mb-2 mb-sm-0 w-100 w-sm-auto"
                  style="max-width: 150px;"
                ></v-select>
                <v-select
                  v-model="editableNodeConfig.state.r2_sensor_type"
                  :items="['gpio', 'i2c']"
                  label="Type"
                  dense outlined hide-details
                  class="mr-sm-2 mb-2 mb-sm-0 w-100 w-sm-auto"
                  style="max-width: 100px;"
                ></v-select>
              </div>
              <div class="d-flex flex-column" style="width: 100%;">
                <span class="text-caption mb-1" :class="{ 'text-disabled': !editableNodeConfig.state.r2_sensor_type }">Pin/Addr:</span>
                <v-btn-toggle
                  v-model="editableNodeConfig.state.r2_sensor_pin"
                  variant="outlined"
                  density="compact"
                  divided
                  :disabled="!editableNodeConfig.state.r2_sensor_type"
                  class="flex-wrap"
                >
                  <v-btn
                    v-for="pin in r2_sensor_pins"
                    :key="pin"
                    :value="pin"
                    class="flex-grow-1"
                  >
                    {{ pin }}
                  </v-btn>
                </v-btn-toggle>
              </div>
              <div class="d-flex flex-column" style="width: 100%;">
                <v-text-field
                  v-if="editableNodeConfig.state.r2_sensor && editableNodeConfig.state.r2_sensor_type && editableNodeConfig.state.r2_sensor_type == 'i2c' && editableNodeConfig.state.r2_sensor_pin"
                  v-model="editableNodeConfig.state.r2_condition_name"
                  label="Work Condition Ex: >30"
                  dense
                  outlined
                  hide-details
                  class="mt-4"
                ></v-text-field>
                <v-btn-toggle
                  v-if="editableNodeConfig.state.r2_sensor && editableNodeConfig.state.r2_sensor_type && editableNodeConfig.state.r2_sensor_type == 'gpio' && editableNodeConfig.state.r2_sensor_pin"
                  v-model="editableNodeConfig.state.r2_condition_name"
                  variant="outlined"
                  density="compact"
                  divided
                  class="mt-4"
                >
                  <v-btn :value="true">TRUE</v-btn>
                  <v-btn :value="false">FAlSE</v-btn>
                </v-btn-toggle>
              </div>
            </div>
          </v-list-item>
        </v-list>
        
        <v-card-actions class="pa-3">
          <v-btn
            v-if="selectedNode.node_type === 'sensor'"
            style="background-color: #a2af9b; color: black"
            text
            @click="saveNodeConfig"
          >
            Save
          </v-btn>
          <v-btn
            v-if="selectedNode.node_type === 'relay' && !selectedNode.config"
            style="background-color: #a2af9b; color: black"
            text
            @click="saveNodeConfig"
          >
            Save
          </v-btn>
          <v-btn
            v-if="selectedNode.node_type === 'relay' && selectedNode.config"
            style="background-color: rgb(211, 59, 59); color: white"
            text
            @click="deleteConfig(selectedNode)"
          >
            Delete
          </v-btn>
          <v-spacer></v-spacer>
          <v-btn 
            style="background-color: #90a4ae; color: black" 
            text
            @click="closeDialog"
          >
            Close
          </v-btn>
        </v-card-actions>
      </v-card>
    </v-dialog>
    <v-snackbar
      v-model="snackbar"
      top
      absolute
      :timeout="notify_timeout"
      :color="notify_color"
      elevation="5"
    >
      {{ message }}
      <template v-slot:actions>
        <v-btn text @click="snackbar = false">Close</v-btn>
      </template>
    </v-snackbar>
  </div>
</template>
<script>
export default {
  name: "NodeManager",
  mounted() {
    if (localStorage.getItem("loggedIn") === "false") {
      this.redirectLogin();
    }
    this.getNodesMongoDB();
    this.checkNodeStatus(); 
    this.checkMasterStatus();
    this.immediatePing();

  },
  data() {
    return {
      slave_nodes: [],
      getNode_request: false,
      ping_delay: 0,
      allNode_request: false,
      activeNode_request: false,
      unpair_request: false,
      gpioConfig_request: false,
      relayConfig_request: false,
      removeConfig_request: false,
      toAuto_request: false,
      toManual_request: false,
      activeNodeSave_request: false,
      get_gpio_type_request: false,
      isLoading: false,
      master_status: "offline",
      editingNodeMac: null,
      editingNodeName: "",
      dialog: false,
      selectedNode: null,
      editableNodeConfig: null, // Temporary storage for edits in the dialog
      snackbar: false,
      message: '',
      notify_color: "teal",
      notify_timeout: 0,
      gpio_config: {
        node_id: "",
        mode: {}
      },
      loadingNodeId: null, // To track loading state for sensor config dialog
      offline_buffer: false,
    };
  },
  computed: {
    // Returns a list of online sensor nodes for the dropdowns
    sensorNodeList() {
      return this.slave_nodes
        .filter(n => n.node_type === 'sensor' && n.node_status === 'online')
        .map(n => n.name || n.mac_address);
    },
    // Dynamically gets the available pins for the Relay 1 sensor
    r1_sensor_pins() {
      if (!this.editableNodeConfig || !this.editableNodeConfig.state.r1_sensor || !this.editableNodeConfig.state.r1_sensor_type) {
        return [];
      }
      
      const selectedNodeName = this.editableNodeConfig.state.r1_sensor;
      const node = this.slave_nodes.find(n => (n.name === selectedNodeName || n.mac_address === selectedNodeName));
      
      if (!node || !node.state) return [];
      const type = this.editableNodeConfig.state.r1_sensor_type;
      if (type === 'gpio' && node.state.gpio) {
        return Object.keys(node.state.gpio);
      }
      if (type === 'i2c' && node.state.i2c) { // Assuming i2c data might exist at node.state.i2c
        return Object.keys(node.state.i2c);
      }
      
      return [];
    },
    // Dynamically gets the available pins for the Relay 2 sensor
    r2_sensor_pins() {
      if (!this.editableNodeConfig || !this.editableNodeConfig.state.r2_sensor || !this.editableNodeConfig.state.r2_sensor_type) {
        return [];
      }
      
      const selectedNodeName = this.editableNodeConfig.state.r2_sensor;
      const node = this.slave_nodes.find(n => (n.name === selectedNodeName || n.mac_address === selectedNodeName));
      
      if (!node || !node.state) return [];
      const type = this.editableNodeConfig.state.r2_sensor_type;
      if (type === 'gpio' && node.state.gpio) {
        return Object.keys(node.state.gpio);
      }
      if (type === 'i2c' && node.state.i2c) {
        return Object.keys(node.state.i2c);
      }
      
      return [];
    }
  },
  watch: {
    msg(newMsg) {
      if (newMsg.type == "RELAY_AUTO_UPDATE") {
        // auto update relay state
        this.slave_nodes = this.slave_nodes.map(node => {
          if (node.node_id === newMsg.payload.node_id && node.manual_mode == false) {
            return {
              ...node,
              state: {
                ...node.state,
                ...newMsg.payload.state
              }
            };
          }
          return node;
        });
      }
      if (newMsg.type == "MONGO_DB" && this.getNode_request) {
        if (newMsg.payload == "alive") {
          this.slave_nodes = [];
          this.getNode_request = false;
          this.isLoading = false;
          return;
        }
        this.isLoading = false;
        this.slave_nodes = Object.values(newMsg.payload).map(node => ({
          ...node,
          node_status: "offline",
          manual_mode: false,
        }));
        this.getNode_request = false;
      }
      if (newMsg.type == "PONG") {
        if (this.offline_buffer == false) {
          this.offline_buffer = true;
          const sendObj = { payload: "active", type: "ACTIVE_NODE" };
          this.activeNode_request = true;
          this.send(sendObj);
        }
        this.master_status = "online";
        if (this.ping_delay < 2) {
          this.ping_delay += 1;
        }
      }
      if (newMsg.type == "ACTIVE_NODE" && this.activeNode_request) {
        this.activeNode_request = false;
        this.slave_nodes = this.slave_nodes.map(node => {
          const key = node.mac_address;
          if (newMsg.payload[key] && newMsg.payload[key].state) {
            if (node.manual_mode == false) {
              return {
                ...node,
                state: newMsg.payload[key]?.state || {},
                node_status: "online"
              };
            } else {
              return {
                ...node,
                node_status: "online"
              };
            }
          } else {
            return {
              ...node,
              node_status: "offline"
            };
          }
        });
      }

      if (newMsg.type == "ACTIVE_NODE" && this.activeNodeSave_request) {
        this.activeNodeSave_request = false;
        this.slave_nodes = this.slave_nodes.map(node => {
          const key = node.mac_address;
          if (newMsg.payload[key] && newMsg.payload[key].state) {
            if (node.manual_mode == false) {
              return {
                ...node,
                state: newMsg.payload[key]?.state || {},
                node_status: "online"
              };
            } else {
              return {
                ...node,
                node_status: "online"
              };
            }
          } else {
            return {
              ...node,
              node_status: "offline"
            };
          }
        });
        for (const key in newMsg.payload) {
          const updateObj = {
            collection: "nodes",
            operation: "updateOne",
            type: "MONGO_DB",
            payload: [
              { mac_address: key },
              { $set: newMsg.payload[key] }
            ]
          };
          this.send(updateObj);
        }
        setTimeout(() => {
          this.refreshPage();
        }, 5000);
      }
      if (newMsg.type == "ALL_NODE" && this.allNode_request) {
        this.allNode_request = false;
        const dropObj = {
          collection: "nodes",
          payload: {},
          operation: "deleteMany",
          type: "MONGO_DB"
        };
        this.send(dropObj);
        setTimeout(() => {
          const insertObj = {
            collection: "nodes",
            payload: [Object.values(newMsg.payload)],
            operation: "insertMany",
            type: "MONGO_DB"
          };
          this.send(insertObj);
          this.showNotification(`Sync succeeded`, "green");
          this.saveActiveNode();
          this.activeNodeSave_request = true;
          this.getNodesMongoDB();
        }, 2000);
      }
      
      if (newMsg.type == "UNPAIR_NODE" && this.unpair_request) {
        const deleteObj = {
          collection: "nodes",
          operation: "deleteOne",
          type: "MONGO_DB",
          payload: [{ mac_address: newMsg.payload.mac_address }]
        };
        this.send(deleteObj);
        this.slave_nodes = this.slave_nodes.filter(
          (n) => n.mac_address !== newMsg.payload.mac_address
        );
        this.unpair_request = false;
        this.showNotification(`Unpair ${newMsg.payload.mac_address} succeeded`, "green");
        // remove detected node paired
        let data = JSON.parse(localStorage.getItem("nodes_detected")) || [];
        data = data.filter(n => n.mac_address !== newMsg.payload.mac_address);
        localStorage.setItem("nodes_detected", JSON.stringify(data));
      }
      if (newMsg.type == "UPDATE_GPIO_CONFIG" && this.gpioConfig_request) {
        this.gpioConfig_request = false;
        this.showNotification(`node_id : ${newMsg.payload.node_id} GPIO configuration updated successfully`, "green");
      }
      if (newMsg.type == "RELAY_CONFIG" && this.relayConfig_request) {
        this.relayConfig_request = false;
        this.showNotification(`node_id : ${newMsg.payload.node_id} Relay configuration updated successfully`, "green");
        this.saveActiveNode();
        this.activeNodeSave_request = true;
        this.refreshPage();
      }
      if (newMsg.type == "REMOVE_CONFIG" && this.removeConfig_request) {
        this.removeConfig_request = false;
        this.showNotification(`node_id : ${newMsg.payload.node_id} configuration removed successfully`, "green");
        this.saveActiveNode();
        this.activeNodeSave_request = true;
        this.refreshPage();
      }
      if (newMsg.type == "REMOVE_CONFIG" && this.toManual_request) {
        this.toManual_request = false;
        // change auto mode to manual mode
        this.slave_nodes = this.slave_nodes.map(node => {
          if (node.node_id === newMsg.payload.node_id) {
            return {
              ...node,
              manual_mode: true
            };
          }
          return node;
        });
      }
      if (newMsg.type == "RELAY_CONFIG" && this.toAuto_request) {
        this.toAuto_request = false;
        // change manual mode to auto mode
        this.slave_nodes = this.slave_nodes.map(node => {
          if (node.node_id === newMsg.payload.node_id) {
            return {
              ...node,
              manual_mode: false
            };
          }
          return node;
        }); 
      }
      if (newMsg.type == "GET_GPIO_TYPE" && this.get_gpio_type_request) {
        this.get_gpio_type_request = false;
        if (this.gpio_config && this.gpio_config.node_id === newMsg.payload.node_id) {
          this.gpio_config = {
            node_id: newMsg.payload.node_id,
            mode: newMsg.payload.mode || this.gpio_config.mode
          };
        }
        this.loadingNodeId = null; // Stop loading
      }
    }
  },
  methods: {
    saveNodeConfig() {
      if (!this.selectedNode) return;
      if (this.selectedNode.node_type === 'sensor') {
        this.saveSensorConfig();
      } else if (this.selectedNode.node_type === 'relay') {
        this.saveRelayConfig();
      }
    },
    saveRelayConfig() {
      // Ensure there is a node configuration to save
      if (!this.editableNodeConfig) return;

      // Find the corresponding sensor nodes for each relay
      const r1_sensor_node = this.slave_nodes.find(n => 
        n.name === this.editableNodeConfig.state.r1_sensor || n.mac_address === this.editableNodeConfig.state.r1_sensor
      );
      const r2_sensor_node = this.slave_nodes.find(n => 
        n.name === this.editableNodeConfig.state.r2_sensor || n.mac_address === this.editableNodeConfig.state.r2_sensor
      );

      // Helper function to safely build the condition string
      const buildCondition = (type, pin, value) => {
        // Return null if any required part of the condition is missing
        if (!type || !pin || value === undefined || value === null || value === '') {
          return null;
        }
        // Format the condition string based on the sensor type
        return type === "gpio" ? `${pin}|${value}` : `${pin}${value}`;
      };

      const configObj = {
        payload: {
          [this.editableNodeConfig.node_id]: {
            r1: {
              node_id: r1_sensor_node ? r1_sensor_node.node_id : null,
              condition: buildCondition(
                this.editableNodeConfig.state.r1_sensor_type,
                this.editableNodeConfig.state.r1_sensor_pin,
                this.editableNodeConfig.state.r1_condition_name
              )
            },
            r2: {
              node_id: r2_sensor_node ? r2_sensor_node.node_id : null,
              condition: buildCondition(
                this.editableNodeConfig.state.r2_sensor_type,
                this.editableNodeConfig.state.r2_sensor_pin,
                this.editableNodeConfig.state.r2_condition_name
              )
            }
          }
        },
        type: "RELAY_CONFIG",
      };
      this.send(configObj);
      this.showNotification("Saving relay configuration...", "blue");
      this.relayConfig_request = true;
      this.closeDialog();
    },
    saveSensorConfig() {
      const updateObj = {
        payload: this.gpio_config,
        type: "UPDATE_GPIO_CONFIG",
      };
      this.send(updateObj);
      this.gpioConfig_request = true;
      this.showNotification("Saving GPIO configuration...", "blue");
      this.closeDialog();
    },
    closeDialog() {
      this.dialog = false;
      this.loadingNodeId = null; // Reset loading state on close
      this.gpio_config = {
        node_id: "",
        mode: {}
      };
      this.selectedNode = null;
      this.editableNodeConfig = null;
    },
    updateGpioConfig(gpioPin, newType) {
      this.gpio_config.mode[gpioPin] = newType;
    },
    showNotification(msg = 'Notification shown at the top!', color = "teal") {
      this.notify_timeout += 3000;
      this.message = msg;
      this.notify_color = color;
      this.snackbar = true;
      this.$emit('send', { payload: 'top_notification_shown' });
    },
    unpairNode(node) {
      if (this.master_status !== "online") {
        this.showNotification("Master node is offline. Cannot fetch relay configurations.", "red");
        return;
      }
      if (node.node_status == "online") {
        if (confirm(`Are you sure you want to unpair the node "${node.name || node.mac_address}"?`)) {
          const unpairObj = {
            payload: node.node_id,
            type: "UNPAIR_NODE",
          }
          this.send(unpairObj);
          this.unpair_request = true;
          this.showNotification("Unpairing...", "blue");
        }
      } else {
        this.showNotification(`This ${node.mac_address} is offline`, "red");
      }
    },
    setManualMode(node) {
      if (!node.manual_mode) {
        const removeObj = {
          payload: node.node_id,
          type: "REMOVE_CONFIG",
        }
        this.send(removeObj);
        this.toManual_request = true;
        this.showNotification(`Toggle ${node.name || node.mac_address} to manual....`, "blue");
      } else {
        if (node.config) {
          const configObj = {
            payload: {
              [node.node_id]: {
                ...node.config
              }
            },
            type: "RELAY_CONFIG",
          };
          this.send(configObj);
          this.toAuto_request = true;
          this.showNotification(`Toggle ${node.name || node.mac_address} to auto....`, "blue");
        } else {
          this.showNotification("No configuration found for this relay node. Please set up the configuration first.", "red");
          return;
        }
      }
    },
    toggleRelay(node) {
      if (!node || !node.state) {
        return;
      }
      const commandObj = {
        type: "RELAY_CONTROL",
        payload: {
          node_id: node.node_id,
          state: {
            "r1": node.state.r1,
            "r2": node.state.r2,
          },
        }
      };
      this.send(commandObj);
    },
    showNodePopup(node) {
      this.selectedNode = node;
      this.editableNodeConfig = JSON.parse(JSON.stringify(node));
      this.dialog = true;

      if (node.node_type == "sensor") {
        this.loadingNodeId = node.node_id; // Start loading

        // set default gpio config mode to button
        let default_gpio = node.state.gpio || {};
        for (let key in default_gpio) {
          default_gpio[key] = "button";
        } 
        this.gpio_config = {
          node_id: node.node_id,
          mode: default_gpio || {}
        };
        
        const getObj = {
          payload: node.node_id,
          type: "GET_GPIO_TYPE",
        }
        this.send(getObj);
        this.get_gpio_type_request = true;
      }
    },
    startEditing(node) {
      this.editingNodeMac = node.mac_address;
      this.editingNodeName = node.name || '';
    },
    saveNodeName() {
      if (!this.editingNodeMac) return;
      const nodeToUpdate = this.slave_nodes.find(n => n.mac_address === this.editingNodeMac);
      if (nodeToUpdate) {
        nodeToUpdate.name = this.editingNodeName;
        const updateObj = {
          collection: "nodes",
          operation: "updateOne",
          type: "MONGO_DB",
          payload: [
            { mac_address: this.editingNodeMac },
            { $set: { name: this.editingNodeName } }
          ]
        };
        this.send(updateObj);
      }
      this.editingNodeMac = null;
      this.editingNodeName = "";
    },
    redirectLogin() {
      const base = window.location.origin;
      window.location.href = `${base}/dashboard/login`;
    },
    getNodesMongoDB() {
      const sendObj = {
        collection: "nodes",
        operation: "find.toArray",
        type: "MONGO_DB"
      };
      this.getNode_request = true;
      this.send(sendObj);
    },
    checkMasterStatus() {
      setInterval(() => {
        if (this.ping_delay > 0) {
          this.ping_delay -= 1;
        } else {
          this.master_status = "offline";
          this.offline_buffer = true;
          this.slave_nodes = this.slave_nodes.map(node => ({
            ...node,
            node_status: "offline"
          }));
        }
      }, 10000);
    },
    checkNodeStatus() {
      const sendObj = { payload: "active", type: "ACTIVE_NODE" };
      setInterval(() => {
        if (this.master_status == "online") {
          this.activeNode_request = true;
          this.send(sendObj);
        }
      }, 10000);
    },
    saveActiveNode() {
      const sendObj = { payload: "active", type: "ACTIVE_NODE" };
      this.activeNodeSave_request = true;
      this.send(sendObj);
    },
    syncSlave() {
      if (this.master_status !== "online") {
        this.showNotification("Master node is offline. Cannot fetch relay configurations.", "red");
        return;
      }
      this.allNode_request = true;
      const sendObj = { payload: "getAllNode", type: "ALL_NODE" };
      this.send(sendObj);
      this.isLoading = true;
    },
    refreshPage() {
      window.location.reload();
    },
    removeConfig(node) {
      const removeObj = {
        payload: node.node_id,
        type: "REMOVE_CONFIG",
      }
      this.send(removeObj);
      this.removeConfig_request = true;
    },
    deleteConfig(node) {
      if (confirm(`Are you sure you want to delete the configuration for the node "${node.name || node.mac_address}"?`)) {
        this.removeConfig(node);
        this.showNotification("Removing configuration...", "blue");
        this.closeDialog();
      }
    },
    immediatePing() {
      const sendObj = { payload: "ping", type: "PING" };
      this.send(sendObj);
    }
  }
}
</script>
<style scoped>
/* Add any component-specific styles here */
</style>
