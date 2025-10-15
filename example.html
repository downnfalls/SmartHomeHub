<template>
  <div style="display: flex; flex-direction: column;" class="ma-3">
    <div class="top-buttons d-lg-flex d-sm-block justify-space-between align-center">
      <strong class="text-h6 font-weight-bold text-uppercase text-blue-grey-darken-4">
        status :
        <span :class="master_status === 'online' ? 'text-teal' : 'text-red-darken-2'">
          {{ master_status }}
        </span>
      </strong>
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

    <!-- Slave Nodes -->
    <v-row class="mt-5 pa-4 rounded-lg" style="background-color: #a2af9b; color: black">
      <v-col
        v-for="(node, index) in slave_nodes"
        :key="index"
        cols="12"
        sm="6"
        md="3"
      >
        <v-sheet class="pa-3 text-subtitle-1 text-blue-grey-darken-4 rounded-lg" elevation="1">
          <v-sheet>
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
                ></v-btn>
                <v-btn
                  icon="mdi-delete"
                  size="small"
                  variant="text"
                  color="red-darken-2"
                  @click.stop="unpairNode(node)"
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
            <p>Node Type : {{ node.node_type }}</p>
            <p>Description : {{ node.description }}</p>

            <div v-if="node.node_type === 'relay' && node.node_status === 'online'">
              <v-btn
                class="mb-2 mt-2"
                block
                color="blue-grey-darken-1"
                @click="setManualMode(node)"
              >
                {{ toggle_mode }}
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

    <!-- GPIO Config Dialog -->
    <v-dialog v-model="dialog" max-width="500">
      <v-card>
        <div
          v-if="selectedNode"
          class="text-h6 text-center text-blue-grey-darken-4 pa-3"
          style="background-color: #a2af9b;"
        >
          {{ selectedNode.name || 'Mesh Node' }}
        </div>
        
        <!-- sensor popup content -->
        <v-list v-if="editableNodeConfig && editableNodeConfig.state && editableNodeConfig.state.gpio">
          <v-list-item 
            v-for="(gpioConfig, gpioPin) in editableNodeConfig.state.gpio" 
            :key="gpioPin"
          >
            <v-list-item-title class="font-weight-bold">
              {{ gpioPin }}
            </v-list-item-title>
            
            <template v-slot:append>
              <v-btn-toggle
                v-model="gpio_config.mode[gpioPin]"
                variant="outlined"
                divided
                density="compact"
              >
                <v-btn
                  value="button"
                  :disabled="gpio_config.mode[gpioPin] === 'button'"
                  :color="gpio_config.mode[gpioPin] === 'button' ? 'teal' : ''"
                  :variant="gpio_config.mode[gpioPin] === 'button' ? 'flat' : 'outlined'"
                  @click="updateGpioConfig(gpioPin, 'button')"
                >
                  Button
                </v-btn>
                <v-btn
                  value="toggle"
                  :disabled="gpio_config.mode[gpioPin] === 'toggle'"
                  :color="gpio_config.mode[gpioPin] === 'toggle' ? 'teal' : ''"
                  :variant="gpio_config.mode[gpioPin] === 'toggle' ? 'flat' : 'outlined'"
                  @click="updateGpioConfig(gpioPin, 'toggle')"
                >
                  Toggle
                </v-btn>
                <v-btn
                  value="delay"
                  :disabled="gpio_config.mode[gpioPin] === 'delay'"
                  :color="gpio_config.mode[gpioPin] === 'delay' ? 'teal' : ''"
                  :variant="gpio_config.mode[gpioPin] === 'delay' ? 'flat' : 'outlined'"
                  @click="updateGpioConfig(gpioPin, 'delay')"
                >
                  Delay
                </v-btn>
              </v-btn-toggle>
            </template>
          </v-list-item>
        </v-list>

        <!-- relay popup content -->
        <v-list v-else-if="editableNodeConfig && editableNodeConfig.state && (editableNodeConfig.node_type === 'relay')">
          <v-list-item>
            <v-list-item-title class="font-weight-bold">
              Relay 1
            </v-list-item-title>
            <template v-slot:append>
              <v-switch
                v-model="editableNodeConfig.state.r1"
                color="teal"
                inset
                hide-details
              ></v-switch>
            </template>
          </v-list-item>
          <v-list-item>
            <v-list-item-title class="font-weight-bold">
              Relay 2
            </v-list-item-title>
            <template v-slot:append>
              <v-switch
                v-model="editableNodeConfig.state.r2"
                color="teal"
                inset
                hide-details
              ></v-switch>
            </template>
          </v-list-item>
        
        <v-card-actions class="pa-3">
          <v-btn
            v-if="selectedNode && selectedNode.node_type === 'sensor'"
            style="background-color: #a2af9b; color: black"
            text
            @click="saveSensorConfig"
          >
            Save
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

    <!-- Notification -->
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
  mounted() {
    if (localStorage.getItem("loggedIn") === "false") {
      this.redirectLogin();
    }
    this.getNodesMongoDB();
    this.checkMasterStatus();
    this.checkNodeStatus();
  },
  data() {
    return {
      slave_nodes: [],
      getNode_request: false,
      ping_request: false,
      allNode_request: false,
      activeNode_request: false,
      unpair_request: false,
      isLoading: false,
      master_status: "offline",
      editingNodeMac: null,
      editingNodeName: "",
      dialog: false,
      selectedNode: null,
      editableNodeConfig: null,
      toggle_mode: "MANUAL",
      snackbar: false,
      message: '',
      notify_color: "teal",
      notify_timeout: 0,
      gpio_config: {
        node_id: "",
        mode: {}
      },
    };
  },
  watch: {
    msg(newMsg) {
      if (newMsg.type == "MONGO_DB" && this.getNode_request) {
        this.isLoading = false;
        this.slave_nodes = Object.values(newMsg.payload).map(node => ({
          ...node,
          node_status: "offline",
          manual_mode: false
        }));
        this.getNode_request = false;
      }

      if (newMsg.type == "PONG" && this.ping_request) {
        this.master_status = "online";
        if (this.ping_delay < 2) {
          this.ping_delay += 2;
        }
        this.ping_request = false;
      }

      if (newMsg.type == "ACTIVE_NODE" && this.activeNode_request) {
        this.activeNode_request = false;
        this.slave_nodes = this.slave_nodes.map(node => {
          const key = node.mac_address;
          if (newMsg.payload[key] && newMsg.payload[key].state) {
            if (!node.manual_mode) {
              return {
                ...node,
                state: newMsg.payload[key]?.state || {},
                node_status: "online"
              };
            } else {
              return { ...node, node_status: "online" };
            }
          } else {
            return { ...node, node_status: "offline" };
          }
        });
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
          this.getNodesMongoDB();
          this.showNotification(`Sync succeeded`, "green");
        }, 1500);
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
          n => n.mac_address !== newMsg.payload.mac_address
        );
        this.unpair_request = false;
        this.showNotification(`Unpair ${newMsg.payload.mac_address} succeeded`, "green");
      }
    }
  },
  methods: {
    closeDialog() {
      this.dialog = false;
      this.gpio_config = { node_id: "", mode: {} };
      this.selectedNode = null;
      this.editableNodeConfig = null;
    },
    updateGpioConfig(gpioPin, newType) {
      this.gpio_config.mode[gpioPin] = newType;
      console.log("Updated GPIO Config:", this.gpio_config);
    },
    saveSensorConfig() {
      if (!this.selectedNode || !this.editableNodeConfig) return;
      const originalNode = this.slave_nodes.find(n => n.mac_address === this.selectedNode.mac_address);
      if (originalNode) {
        originalNode.state.gpio = this.editableNodeConfig.state.gpio;
        const updateObj = {
          collection: "nodes",
          operation: "updateOne",
          type: "MONGO_DB",
          payload: [
            { mac_address: this.selectedNode.mac_address },
            { $set: { "state.gpio": this.editableNodeConfig.state.gpio } }
          ]
        };
        this.send(updateObj);
      }
      this.showNotification(`Configuration for ${this.selectedNode.name || 'sensor'} saved!`, 'green');
      this.dialog = false;
    },
    showNotification(msg = 'Notification shown at the top!', color = "teal") {
      this.notify_timeout += 3000;
      this.message = msg;
      this.notify_color = color;
      this.snackbar = true;
      this.$emit('send', { payload: 'top_notification_shown' });
    },
    unpairNode(node) {
      if (node.node_status == "online") {
        if (confirm(`Are you sure you want to unpair the node "${node.name || node.mac_address}"?`)) {
          const unpairObj = { payload: node.node_id, type: "UNPAIR_NODE" };
          this.send(unpairObj);
          this.unpair_request = true;
          this.showNotification("Unpairing...", "blue");
        }
      } else {
        this.showNotification(`This ${node.mac_address} is offline`, "red");
      }
    },
    setManualMode(node) {
      if (this.toggle_mode == "MANUAL") {
        node.manual_mode = true;
        this.toggle_mode = "AUTO";
      } else {
        node.manual_mode = false;
        this.toggle_mode = "MANUAL";
      }
    },
    toggleRelay(node, relayKey) {
      if (!node || !node.state) return;
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
      if (node.node_type == "sensor") {
        let default_gpio = node.state.gpio || {};
        for (let key in default_gpio) {
          default_gpio[key] = "button";
        } 
        this.gpio_config = {
          node_id: node.node_id,
          mode: default_gpio || {}
        };
        console.log("Default GPIO Config:", this.gpio_config);
      }
      this.selectedNode = node;
      this.editableNodeConfig = JSON.parse(JSON.stringify(node));
      this.dialog = true;
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
      const sendObj = { payload: "ping", type: "PING" };
      this.ping_request = true;
      this.send(sendObj);
      setInterval(() => {
        this.ping_request = true;
        if (this.ping_delay == 0) {
          this.master_status = "offline";
        } else {
          this.master_status = "online";
          this.ping_delay -= 1;
        }
        this.send(sendObj);
      }, 5000);
    },
    checkNodeStatus() {
      const sendObj = { payload: "active", type: "ACTIVE_NODE" };
      setInterval(() => {
        if (this.master_status == "online") {
          this.activeNode_request = true;
          this.send(sendObj);
        }
      }, 1000);
    },
    syncSlave() {
      this.allNode_request = true;
      const sendObj = { payload: "getAllNode", type: "ALL_NODE" };
      this.send(sendObj);
      this.isLoading = true;
    },
    refreshPage() {
      window.location.reload();
    }
  }
};
</script>
