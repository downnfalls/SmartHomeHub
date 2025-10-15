<template>
  <div class="mesh-detail">
    <div class="top-buttons d-lg-flex d-sm-block justify-space-between align-center">
      <strong class="text-h6 font-weight-bold text-uppercase text-blue-grey-darken-4">
          status : 
          <span :class="master_status == 'online' ? 'text-teal' : 'text-red-darken-2'">
            {{ master_status }}
          </span>
        </strong>
      <div>
        <v-btn
          outlined
          @click="scanNodes"
          style="background-color: #a2af9b; color: black"
          class="font-weight-bold text-subtitle-1 text-blue-grey-darken-4 mr-4"
        >
          SCAN
        </v-btn>

        <v-btn
        color="red-darken-1"
        outlined
        @click="deleteAllNodes"
        class="font-weight-bold text-subtitle-1"
      >
        RESET !
      </v-btn>
      </div>
    </div>

    <div class="nodes-container pa-4 rounded-lg" style="background-color: #a2af9b;" >
      <div
        v-for="(node, index) in nodes_detected.filter(n => !n.mesh_slave)"
        :key="index"
        class="node-item"
      >
        <v-dialog
          v-model="node.active"
          transition="dialog-bottom-transition"
          class="w-auto"
          style="max-width: 400px;"
        >
          <template v-slot:activator="{ props: activatorProps }">
            <v-btn
              v-bind="activatorProps"
              class="pt-2 pb-2"
              color="primary"
              outlined
            >
            <div class="d-lg-flex d-xs-block">
              <div class="mr-lg-4">
                Node Name: {{ node.display_name }}
                <br /><br />
                MAC Addr: {{ node.mac_address }}
              </div>
              <v-btn
                color="red-darken-1"
                small
                class=""
                @click.stop="deleteNode(index)"
              >
                Delete
              </v-btn>
            </div>
            </v-btn>
          </template>

          <v-card>
            <v-toolbar title="New Slave" style="background-color: #a2af9b;"></v-toolbar>

            <v-card-text>
              <p class="mb-2"><strong>Node Name:</strong> {{ node.display_name }}</p>
              <p class="mb-2"><strong>MAC Address:</strong> {{ node.mac_address }}</p>
              <p>
                <strong>Description:</strong>
                <v-text-field v-model="description" label="Description" outlined class="mt-2"></v-text-field>
              </p>
            </v-card-text>

            <v-card-actions class="justify-space-between">
              <v-btn
                  style="background-color: #a2af9b; color: black"
                  :loading="isLoading"
                  :disabled="isLoading"
                  @click="addSlave(node)"
                >
                  <template v-slot:loader>
                    <v-progress-circular indeterminate color="white" size="20"></v-progress-circular>
                  </template>
                  <span v-if="!isLoading">Pair</span>
                  <span v-else>Loading...</span>
                </v-btn>

              <v-btn
                @click="cancelAddSlave(node)"
                style="background-color: #90a4ae; color: black"
              >
                Cancel
              </v-btn>
            </v-card-actions>
          </v-card>
        </v-dialog>
      </div>
    </div>
  </div>
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
</template>

<script>
export default {
  props: {
    msg: {
      type: Object,
      required: true
    }
  },
  data() {
    return {
      nodes_detected: [],
      description: "",
      pair_display_name: "",
      pair_mac_addr: "",
      master_status: "offline",
      ping_delay: 0,
      scan_request: false, // filter msg scan
      pair_request: false, // filter msg pair
      isLoading: false,
      snackbar: false, // control notify visibility
      message: '', // message of notify
      notify_color: "teal",
      notify_timeout: 0
    };
  },
  mounted() {
    // invoke logic function
    this.scanNodes();

    // Load saved nodes from localStorage
    const savedNodes = localStorage.getItem("nodes_detected");
    if (savedNodes) {
      try {
        this.nodes_detected = JSON.parse(savedNodes).map(n => ({
          ...n,
          active: false, // ensure active property exists
        }));
      } catch (e) {
        console.error("Failed to parse nodes_detected from localStorage", e);
      }
    }

    if (localStorage.getItem("loggedIn") === "false") {
      this.redirectLogin();
    }

    // Load data from localStorage

    const description = localStorage.getItem("description");
    const pair_display_name = localStorage.getItem("pair_display_name");
    const pair_mac_addr = localStorage.getItem("pair_mac_addr");
    const isLoading = localStorage.getItem("isLoading");

    if (description && description != "") {
        this.description = description;
    }

    if (pair_display_name && pair_display_name != "") {
      this.pair_display_name = pair_display_name;
    }

    if (pair_mac_addr && pair_mac_addr != "") {
      this.pair_mac_addr = pair_mac_addr;
    }
    if (isLoading == "true") {
      this.isLoading = true;
    } else {
      this.isLoading = false;
    }
  },
  watch: {
    msg: {
      handler(newMsg) {
        // node scan res handler
        if (newMsg && this.scan_request) {
          const payload = { ...newMsg.payload };
          payload.result.forEach(item => {
            const node = { 
              ...item, 
              mesh_slave: false, 
              active: false 
            };

            const exists = this.nodes_detected.some(
              n => n.mac_address === node.mac_address && n.display_name === node.display_name
            );

            if (!exists) {
              this.nodes_detected.push(node);
              console.log("New node detected:", node);
            }
          });
          this.scan_request = false; 
        }

        // pong handler
        if (newMsg && newMsg.pong) {
          this.master_status = "online"
          if (this.ping_delay < 2) {
            this.ping_delay += 1;
          }
        }

        // pair handler and if have pair successed it will auto allow
        if (newMsg && newMsg.pair && (this.pair_request || newMsg.payload.mac_address == this.pair_mac_addr)) {
          this.pair_request = false;

          // check pair status
          switch (newMsg.payload.info) {
            case "pairing to new node.":
              // change to loading state
              this.isLoading = true;

              // Save to localStorage
              localStorage.setItem("description", this.description);
              localStorage.setItem("pair_display_name", this.pair_display_name);
              localStorage.setItem("pair_mac_addr", this.pair_mac_addr);
              localStorage.setItem("isLoading", this.isLoading);

              console.log("Info : Wait for pairing..");
              this.showNotification("Info : Wait for pairing..", "blue");
              
              // wait for paired from esp32
              break;
            case "failed to connect to node.":
              console.log("Error : Failed to connect to node!!");
              this.showNotification("Error : Failed to connect to node!!", "red");
              break;
            case "pair success!":
              console.log("Info : Pair succeed!!");
              this.showNotification("Info : Pair succeed!!", "green");

              // save to mongoDB database
              const sendObj = {
                collection: "nodes",
                payload: { 
                  mac_address: newMsg.payload.mac_address,
                  node_id: newMsg.payload.node_id,
                  node_type: newMsg.payload.node_type,
                  description: this.description
                },
                operation: "insertOne",
                toMongo: true
              };

              this.send(sendObj);

              // Set mesh_slave = true so it no longer appears in scan section
              const targetNode = this.nodes_detected.find(
                n => n.mac_address === this.pair_mac_addr && n.display_name === this.pair_display_name
              );
              if (targetNode) {
                targetNode.mesh_slave = true;
                localStorage.setItem("nodes_detected", JSON.stringify(this.nodes_detected));
              }
              this.isLoading = false;
              // reset pairing node info
              this.description = "";
              this.pair_display_name = "";
              this.pair_mac_addr = "";

              // Save to localStorage
              localStorage.setItem("description", this.description);
              localStorage.setItem("pair_display_name", this.pair_display_name);
              localStorage.setItem("pair_mac_addr", this.pair_mac_addr);
              localStorage.setItem("isLoading", this.isLoading);
              break;
            default:
              console.log("Some error occurred!!");
              this.showNotification("Error : Some error occurred!!", "red");
          }
        }
      },
      deep: true
    }
  },
  methods: {
    showNotification(msg = 'Notification shown at the top!', color = "teal") {
      this.notify_timeout += 3000;
      this.message = msg;
      this.notify_color = color;
      this.snackbar = true;
      // Optional: send message to Node-RED backend
      this.$emit('send', { payload: 'top_notification_shown' });
    },
    redirectLogin() {
      const base = window.location.origin;
      window.location.href = `${base}/dashboard/login`;
    },
    deleteNode(index) {
      this.nodes_detected.splice(index, 1);
      localStorage.setItem(
        "nodes_detected",
        JSON.stringify(this.nodes_detected)
      );
    },
    deleteAllNodes() {
      if (confirm("Are you sure you want to delete all nodes?")) {
        this.nodes_detected = [];
        localStorage.removeItem("nodes_detected");
        const sendObj = {
          collection: "nodes",
          operation: "drop",
          toMongo: true
        };
        this.send(sendObj);
      }
    },
    addSlave(node) {
      this.pair_display_name = node.display_name;
      this.pair_mac_addr = node.mac_address;

      // send pairing request
      const sendObj = {
        payload : node.mac_address,
        pair : "true"
      };
      this.pair_request = true;
      
      // handle master offline
      if (this.master_status == "offline") {
        console.log("Error : Master is offline!!");
        this.showNotification("Error : Master is offline!!", "red");
        return;
      }

      if (this.isLoading) {
        console.log("Warning : Cannot pair node while pairing!!");
        this.showNotification("Warning : Cannot pair node while pairing!!", "yellow");
        return;
      }

      this.send(sendObj);

      // Send message to Node-RED backend
      this.$emit('send', { payload: 'button_clicked' });
    },
    scanNodes() {
      const sendObj = {
        payload: "scan"
      };
      this.scan_request = true; 
      this.send(sendObj);
    },
    cancelAddSlave(node) {
      if (this.isLoading == false) {
        // close dialog pop up
        node.active = false;
      } else {
        if (confirm("Do you want to cancel ?\n[ cancel after pairing you have to sync mesh ]")) {
          this.isLoading = false;
          this.pair_display_name = "";
          this.pair_mac_addr = "";
          this.description = "";

          // Save to localStorage
          localStorage.setItem("description", this.description);
          localStorage.setItem("pair_display_name", this.pair_display_name);
          localStorage.setItem("pair_mac_addr", this.pair_mac_addr);
          localStorage.setItem("isLoading", this.isLoading);

          // close dialog pop up
          node.active = false;
        }
      }
    },
    checkMasterStatus() {
      setInterval(() => {
        if (this.ping_delay > 0) {
          this.ping_delay -= 1;
        } else {
          this.master_status = "offline";
        }
      }, 10000);
    },
  },
};
</script>

<style scoped>
.top-buttons {
  margin: 10px;
}

.mesh-detail {
  display: flex;
  flex-direction: column;
  margin: 10px;
  /* Make the main container take up the full viewport height */
  height: calc(100vh - 20px); /* Subtract margins */
}

.nodes-container {
  display: flex;
  flex-wrap: wrap;
  gap: 10px;
  /* Allow this container to grow and fill the remaining vertical space */
  flex-grow: 1; 
  width: 100%; /* Ensure it takes full width of its parent */
}

.node-item {
  display: inline-block;
}

</style>