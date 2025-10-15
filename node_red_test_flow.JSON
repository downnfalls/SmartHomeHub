[
    {
        "id": "849cc3e8cfb02cf5",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Pseudo Scan Node",
        "func": "if (msg.payload) {\n    msg.payload = {\n        \"result\": [\n            {\n                \"mac_address\": \"0C:AD:ER:BE:HU:15\",\n                \"display_name\": \"MeshNode\"\n            },\n            {\n                \"mac_address\": \"10:B5:1B:15:1E:AA\",\n                \"display_name\": \"MeshNode\"\n            }\n        ]\n    };\n    return msg\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 1030,
        "y": 2120,
        "wires": [
            [
                "3c898bbf7c3df719"
            ]
        ]
    },
    {
        "id": "50d414316f279371",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Pseudo Ping Pong",
        "func": "if (msg.payload) {\n    msg.payload = \"alive\"\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 990,
        "y": 1400,
        "wires": [
            [
                "70b03f26254364a2"
            ]
        ]
    },
    {
        "id": "718f735a79ab1db4",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt out : req/mesh/scan",
        "info": "",
        "x": 910,
        "y": 2080,
        "wires": []
    },
    {
        "id": "1e28c023bd1c9431",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt in : res/mesh/scan",
        "info": "",
        "x": 1140,
        "y": 2080,
        "wires": []
    },
    {
        "id": "628aaeee6693ac2b",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Scan Req",
        "func": "if (msg.payload == \"scan\") {\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 900,
        "y": 1980,
        "wires": [
            [
                "849cc3e8cfb02cf5"
            ]
        ]
    },
    {
        "id": "70b03f26254364a2",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Pong",
        "func": "msg.scan = false;\nmsg.pong = true;\nmsg.info = false;\nmsg.pair = false;\nreturn msg;",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 1110,
        "y": 1540,
        "wires": [
            [
                "ca478c1c869a4381"
            ]
        ]
    },
    {
        "id": "3c898bbf7c3df719",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Scan Res",
        "func": "msg.scan = true;\nmsg.pong = false;\nmsg.info = false;\nmsg.pair = false;\nreturn msg;",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 1140,
        "y": 1980,
        "wires": [
            [
                "990288c0b07710ec"
            ]
        ]
    },
    {
        "id": "a3bcb49fdd374e06",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Pseudo Pairing",
        "func": "if (msg.payload) {\n    msg.payload = {\n        \"mac_address\":\"0C:AD:ER:BE:HU:15\", \n        \"info\":\"pairing to new node.\"\n    };\n\n    // Error condition\n    // msg.payload = {\n    //     \"mac_address\":\"0C:AD:ER:BE:HU:15\",\n    //     \"info\":\"failed to connect to node.\"\n    // };\n\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 1030,
        "y": 2560,
        "wires": [
            [
                "e3794c8ee1dd02c6"
            ]
        ]
    },
    {
        "id": "e3794c8ee1dd02c6",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Pair Res",
        "func": "msg.scan = false;\nmsg.pong = false;\nmsg.info = false;\nmsg.pair = true;\nreturn msg;",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 1150,
        "y": 2420,
        "wires": [
            [
                "8c89bbddc8ff1946"
            ]
        ]
    },
    {
        "id": "cb049ac75fb69645",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Pair Req",
        "func": "if (msg.pair == \"true\") {\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 910,
        "y": 2420,
        "wires": [
            [
                "a3bcb49fdd374e06"
            ]
        ]
    },
    {
        "id": "c24cce32ce2b54b3",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt out : req/mesh/pair",
        "info": "",
        "x": 930,
        "y": 2520,
        "wires": []
    },
    {
        "id": "67fae7c955747ddb",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt in : res/mesh/pair",
        "info": "",
        "x": 1160,
        "y": 2520,
        "wires": []
    },
    {
        "id": "b17e40686d04010d",
        "type": "ui-template",
        "z": "2c682173eb064de0",
        "group": "2f692e7cb40097ed",
        "page": "",
        "ui": "",
        "name": "Login Template",
        "order": 1,
        "width": 0,
        "height": 14,
        "head": "",
        "format": "<template class=\"bg-black-100\">\n  <div class=\"flex flex-col items-center justify-center h-full p-6\">\n    <h2 class=\"text-2xl font-bold mb-4\">Login</h2>\n    <v-text-field v-model=\"username\" label=\"Username\" outlined></v-text-field>\n    <v-text-field v-model=\"password\" label=\"Password\" type=\"password\" outlined></v-text-field>\n    <p v-if=\"error\" class=\"text-red-500 mt-2\">{{ error }}</p>\n    <v-btn class=\"mt-4\" color=\"primary\" @click=\"login\">Login</v-btn>\n  </div>\n</template>\n\n<script>\n  export default {\n  props: {\n    msg: {\n      type: Object,\n      required: true\n    }\n  },\n  data() {\n    return {\n      username: \"\",\n      password: \"\",\n      error: \"\"\n    };\n  },\n  mounted() {\n    if (localStorage.getItem(\"loggedIn\") == \"true\") {\n      this.redirectScan();\n    }\n  },\n  watch: {\n    msg(newMsg) {\n      if (newMsg && newMsg.payload) {\n        const user = newMsg.payload;\n        if (this.username === user.username && this.password === user.password) {\n          this.error = \"\";\n          localStorage.setItem(\"loggedIn\", \"true\");\n          this.redirectScan();\n        } else {\n          this.error = \"Invalid username or password\";\n        }\n      }\n    }\n  },\n  methods: {\n    login() {\n      const sendObj = {\n        collection: \"users\",\n        payload: { username: this.username },\n        operation: \"findOne\"\n      };\n      this.send(sendObj);\n    },\n    redirectScan() {\n      // tell Node-RED to switch tab via ui-control\n      const base = window.location.origin;\n      window.location.href = `${base}/dashboard/scan`;\n    }\n  }\n};\n</script>",
        "storeOutMessages": true,
        "passthru": true,
        "resendOnRefresh": true,
        "templateScope": "local",
        "className": "",
        "x": 1720,
        "y": 1920,
        "wires": [
            [
                "414b2c245a73e2f4"
            ]
        ]
    },
    {
        "id": "f3bda6c28c4cdefd",
        "type": "ui-template",
        "z": "2c682173eb064de0",
        "group": "8146a75c4508b858",
        "page": "",
        "ui": "",
        "name": "Scan Template",
        "order": 1,
        "width": 0,
        "height": 14,
        "head": "",
        "format": "<template>\n  <div class=\"mesh-detail\">\n    <!-- Top action buttons: one left, one right -->\n    <div class=\"top-buttons d-lg-flex d-sm-block justify-space-between align-center\">\n      <strong class=\"text-h6 font-weight-bold text-uppercase text-blue-grey-darken-4\">\n          status : \n          <span :class=\"master_status === 'online' ? 'text-teal' : 'text-red-darken-2'\">\n            {{ master_status }}\n          </span>\n        </strong>\n      <div>\n        <v-btn\n          outlined\n          @click=\"scanNodes\"\n          style=\"background-color: #a2af9b; color: black\"\n          class=\"font-weight-bold text-subtitle-1 text-blue-grey-darken-4 mr-4\"\n        >\n          SCAN\n        </v-btn>\n\n        <v-btn\n        color=\"red-darken-1\"\n        outlined\n        @click=\"deleteAllNodes\"\n        class=\"font-weight-bold text-subtitle-1\"\n      >\n        RESET !\n      </v-btn>\n      </div>\n    </div>\n\n    <!-- Nodes displayed inline (only mesh_slave: false) -->\n    <div class=\"nodes-container pa-4 rounded-lg\" style=\"background-color: #a2af9b;\">\n      <div\n        v-for=\"(node, index) in nodes_detected.filter(n => !n.mesh_slave)\"\n        :key=\"index\"\n        class=\"node-item\"\n      >\n        <!-- Dialog wrapping node button -->\n        <v-dialog\n          v-model=\"node.active\"\n          transition=\"dialog-bottom-transition\"\n          class=\"w-auto\"\n          style=\"max-width: 400px;\"\n        >\n          <template v-slot:activator=\"{ props: activatorProps }\">\n            <v-btn\n              v-bind=\"activatorProps\"\n              class=\"pt-2 pb-2\"\n              color=\"primary\"\n              outlined\n            >\n            <div class=\"d-lg-flex d-xs-block\">\n              <div class=\"mr-lg-4\">\n                Node Name: {{ node.display_name }}\n                <br /><br />\n                MAC Addr: {{ node.mac_address }}\n              </div>\n              <!-- Delete button stays on card -->\n              <v-btn\n                color=\"red-darken-1\"\n                small\n                class=\"\"\n                @click.stop=\"deleteNode(index)\"\n              >\n                Delete\n              </v-btn>\n            </div>\n            </v-btn>\n          </template>\n\n          <!-- Popup content -->\n          <v-card>\n            <v-toolbar title=\"New Slave\" style=\"background-color: #a2af9b;\"></v-toolbar>\n\n            <v-card-text>\n              <p class=\"mb-2\"><strong>Node Name:</strong> {{ node.display_name }}</p>\n              <p class=\"mb-2\"><strong>MAC Address:</strong> {{ node.mac_address }}</p>\n              <p>\n                <strong>Description:</strong>\n                <v-text-field v-model=\"description\" label=\"Description\" outlined class=\"mt-2\"></v-text-field>\n              </p>\n            </v-card-text>\n\n            <v-card-actions class=\"justify-space-between\">\n              <v-btn\n                  style=\"background-color: #a2af9b; color: black\"\n                  :loading=\"isLoading\"\n                  :disabled=\"isLoading\"\n                  @click=\"addSlave(node)\"\n                >\n                  <template v-slot:loader>\n                    <v-progress-circular indeterminate color=\"white\" size=\"20\"></v-progress-circular>\n                  </template>\n                  <span v-if=\"!isLoading\">Pair</span>\n                  <span v-else>Loading...</span>\n                </v-btn>\n\n              <v-btn\n                @click=\"cancelAddSlave(node)\"\n                style=\"background-color: #90a4ae; color: black\"\n              >\n                Cancel\n              </v-btn>\n            </v-card-actions>\n          </v-card>\n        </v-dialog>\n      </div>\n    </div>\n  </div>\n  <!-- Top notification -->\n  <v-snackbar\n    v-model=\"snackbar\"\n    top\n    absolute\n    :timeout=\"notify_timeout\"\n    :color=\"notify_color\"\n    elevation=\"5\"\n  >\n    {{ message }}\n    <template v-slot:actions>\n      <v-btn text @click=\"snackbar = false\">Close</v-btn>\n    </template>\n  </v-snackbar>\n</template>\n\n<script>\nexport default {\n  props: {\n    msg: {\n      type: Object,\n      required: true\n    }\n  },\n  data() {\n    return {\n      nodes_detected: [],\n      description: \"\",\n      pair_display_name: \"\",\n      pair_mac_addr: \"\",\n      master_status: \"offline\",\n      ping_delay: 0,\n      scan_request: false, // filter msg scan\n      pair_request: false, // filter msg pair\n      isLoading: false,\n      snackbar: false, // control notify visibility\n      message: '', // message of notify\n      notify_color: \"teal\",\n      notify_timeout: 0\n    };\n  },\n  mounted() {\n    // invoke logic function\n    this.scanNodes();\n\n    // Load saved nodes from localStorage\n    const savedNodes = localStorage.getItem(\"nodes_detected\");\n    if (savedNodes) {\n      try {\n        this.nodes_detected = JSON.parse(savedNodes).map(n => ({\n          ...n,\n          active: false, // ensure active property exists\n        }));\n      } catch (e) {\n        console.error(\"Failed to parse nodes_detected from localStorage\", e);\n      }\n    }\n\n    if (localStorage.getItem(\"loggedIn\") === \"false\") {\n      this.redirectLogin();\n    }\n\n    // Load data from localStorage\n\n    const description = localStorage.getItem(\"description\");\n    const pair_display_name = localStorage.getItem(\"pair_display_name\");\n    const pair_mac_addr = localStorage.getItem(\"pair_mac_addr\");\n    const isLoading = localStorage.getItem(\"isLoading\");\n\n    if (description && description != \"\") {\n        this.description = description;\n    }\n\n    if (pair_display_name && pair_display_name != \"\") {\n      this.pair_display_name = pair_display_name;\n    }\n\n    if (pair_mac_addr && pair_mac_addr != \"\") {\n      this.pair_mac_addr = pair_mac_addr;\n    }\n    if (isLoading == \"true\") {\n      this.isLoading = true;\n    } else {\n      this.isLoading = false;\n    }\n  },\n  watch: {\n    msg: {\n      handler(newMsg) {\n        // node scan res handler\n        if (newMsg && newMsg.scan && this.scan_request) {\n          const payload = { ...newMsg.payload };\n          payload.result.forEach(item => {\n            const node = { \n              ...item, \n              mesh_slave: false, \n              active: false \n            };\n\n            const exists = this.nodes_detected.some(\n              n => n.mac_address === node.mac_address && n.display_name === node.display_name\n            );\n\n            if (!exists) {\n              this.nodes_detected.push(node);\n              console.log(\"New node detected:\", node);\n            }\n          });\n          this.scan_request = false; \n        }\n\n        // pong handler\n        if (newMsg && newMsg.pong) {\n          this.master_status = \"online\"\n          if (this.ping_delay < 2) {\n            this.ping_delay += 2;\n          }\n        }\n\n        // pair handler and if have pair successed it will auto allow\n        if (newMsg && newMsg.pair && (this.pair_request || newMsg.payload.mac_address == this.pair_mac_addr)) {\n          this.pair_request = false;\n\n          // check pair status\n          switch (newMsg.payload.info) {\n            case \"pairing to new node.\":\n              // change to loading state\n              this.isLoading = true;\n\n              // Save to localStorage\n              localStorage.setItem(\"description\", this.description);\n              localStorage.setItem(\"pair_display_name\", this.pair_display_name);\n              localStorage.setItem(\"pair_mac_addr\", this.pair_mac_addr);\n              localStorage.setItem(\"isLoading\", this.isLoading);\n\n              console.log(\"Info : Wait for pairing..\");\n              this.showNotification(\"Info : Wait for pairing..\", \"blue\");\n              \n              // wait for paired from esp32\n              break;\n            case \"failed to connect to node.\":\n              console.log(\"Error : Failed to connect to node!!\");\n              this.showNotification(\"Error : Failed to connect to node!!\", \"red\");\n              break;\n            case \"pair success!\":\n              console.log(\"Info : Pair succeed!!\");\n              this.showNotification(\"Info : Pair succeed!!\", \"green\");\n\n              // save to mongoDB database\n              const sendObj = {\n                collection: \"nodes\",\n                payload: { \n                  mac_address: newMsg.payload.mac_address,\n                  node_id: newMsg.payload.node_id,\n                  node_type: newMsg.payload.node_type,\n                  description: this.description\n                },\n                operation: \"insertOne\",\n                toMongo: true\n              };\n\n              this.send(sendObj);\n\n              // Set mesh_slave = true so it no longer appears in scan section\n              const targetNode = this.nodes_detected.find(\n                n => n.mac_address === this.pair_mac_addr && n.display_name === this.pair_display_name\n              );\n              if (targetNode) {\n                targetNode.mesh_slave = true;\n                localStorage.setItem(\"nodes_detected\", JSON.stringify(this.nodes_detected));\n              }\n              this.isLoading = false;\n              // reset pairing node info\n              this.description = \"\";\n              this.pair_display_name = \"\";\n              this.pair_mac_addr = \"\";\n\n              // Save to localStorage\n              localStorage.setItem(\"description\", this.description);\n              localStorage.setItem(\"pair_display_name\", this.pair_display_name);\n              localStorage.setItem(\"pair_mac_addr\", this.pair_mac_addr);\n              localStorage.setItem(\"isLoading\", this.isLoading);\n              break;\n            default:\n              console.log(\"Some error occurred!!\");\n              this.showNotification(\"Error : Some error occurred!!\", \"red\");\n          }\n        }\n      },\n      deep: true\n    }\n  },\n  methods: {\n    showNotification(msg = 'Notification shown at the top!', color = \"teal\") {\n      this.notify_timeout += 3000;\n      this.message = msg;\n      this.notify_color = color;\n      this.snackbar = true;\n      // Optional: send message to Node-RED backend\n      this.$emit('send', { payload: 'top_notification_shown' });\n    },\n    redirectLogin() {\n      const base = window.location.origin;\n      window.location.href = `${base}/dashboard/login`;\n    },\n    deleteNode(index) {\n      this.nodes_detected.splice(index, 1);\n      localStorage.setItem(\n        \"nodes_detected\",\n        JSON.stringify(this.nodes_detected)\n      );\n    },\n    deleteAllNodes() {\n      if (confirm(\"Are you sure you want to delete all nodes?\")) {\n        this.nodes_detected = [];\n        localStorage.removeItem(\"nodes_detected\");\n        const sendObj = {\n          collection: \"nodes\",\n          operation: \"drop\",\n          toMongo: true\n        };\n        this.send(sendObj);\n      }\n    },\n    addSlave(node) {\n      this.pair_display_name = node.display_name;\n      this.pair_mac_addr = node.mac_address;\n\n      // send pairing request\n      const sendObj = {\n        payload : node.mac_address,\n        pair : \"true\"\n      };\n      this.pair_request = true;\n      \n      // handle master offline\n      if (this.master_status == \"offline\") {\n        console.log(\"Error : Master is offline!!\");\n        this.showNotification(\"Error : Master is offline!!\", \"red\");\n        return;\n      }\n\n      if (this.isLoading) {\n        console.log(\"Warning : Cannot pair node while pairing!!\");\n        this.showNotification(\"Warning : Cannot pair node while pairing!!\", \"yellow\");\n        return;\n      }\n\n      this.send(sendObj);\n\n      // Send message to Node-RED backend\n      this.$emit('send', { payload: 'button_clicked' });\n    },\n    scanNodes() {\n      const sendObj = {\n        payload: \"scan\"\n      };\n      this.scan_request = true; \n      this.send(sendObj);\n    },\n    cancelAddSlave(node) {\n      if (this.isLoading == false) {\n        // close dialog pop up\n        node.active = false;\n      } else {\n        if (confirm(\"Do you want to cancel ?\\n[ cancel after pairing you have to sync mesh ]\")) {\n          this.isLoading = false;\n          this.pair_display_name = \"\";\n          this.pair_mac_addr = \"\";\n          this.description = \"\";\n\n          // Save to localStorage\n          localStorage.setItem(\"description\", this.description);\n          localStorage.setItem(\"pair_display_name\", this.pair_display_name);\n          localStorage.setItem(\"pair_mac_addr\", this.pair_mac_addr);\n          localStorage.setItem(\"isLoading\", this.isLoading);\n\n          // close dialog pop up\n          node.active = false;\n        }\n      }\n    }\n  },\n};\n</script>\n\n<style scoped>\n.top-buttons {\n  margin: 10px;\n}\n\n.mesh-detail {\n  display: flex;\n  flex-direction: column;\n  margin: 10px;\n}\n\n.nodes-container {\n  display: flex;\n  flex-wrap: wrap;\n  gap: 10px;\n}\n\n.node-item {\n  display: inline-block;\n}\n\n</style>",
        "storeOutMessages": true,
        "passthru": true,
        "resendOnRefresh": true,
        "templateScope": "local",
        "className": "",
        "x": 1020,
        "y": 1800,
        "wires": [
            [
                "4d73b1b5c177d13a"
            ]
        ]
    },
    {
        "id": "81d74a33ea5da797",
        "type": "ui-template",
        "z": "2c682173eb064de0",
        "group": "dd337cb0663d76c3",
        "page": "",
        "ui": "",
        "name": "Home Template",
        "order": 1,
        "width": 0,
        "height": 14,
        "head": "",
        "format": "<template>\n  <div style=\"display: flex; flex-direction: column\" class=\"ma-3\">\n    <div class=\"top-buttons d-lg-flex d-sm-block justify-space-between align-center\">\n      <div>\n        <strong class=\"text-h6 font-weight-bold text-uppercase text-blue-grey-darken-4 mr-4\">\n          status :\n          <span :class=\"master_status === 'online' ? 'text-teal' : 'text-red-darken-2'\">\n            {{ master_status }}\n          </span>\n        </strong>\n        <v-btn\n            outlined\n            style=\"background-color: #a2af9b; color: black\"\n            class=\"font-weight-bold text-subtitle-1 text-blue-grey-darken-4\"\n            @click=\"updateConfigRelay\"\n          >\n          <v-icon>mdi-refresh</v-icon>\n        </v-btn>\n      </div>\n      <div>\n        <v-btn\n          outlined\n          @click=\"refreshPage\"\n          style=\"background-color: #a2af9b; color: black\"\n          class=\"font-weight-bold text-subtitle-1 text-blue-grey-darken-4 mr-4\"\n        >\n          REFRESH\n        </v-btn>\n        <v-btn\n          outlined\n          style=\"background-color: #a2af9b; color: black\"\n          class=\"font-weight-bold text-subtitle-1 text-blue-grey-darken-4\"\n          :loading=\"isLoading\"\n          :disabled=\"isLoading\"\n          @click=\"syncSlave\"\n        >\n          <template v-slot:loader>\n            <v-progress-circular indeterminate color=\"white\" size=\"20\"></v-progress-circular>\n          </template>\n          <span v-if=\"!isLoading\">SYNC</span>\n          <span v-else>Loading...</span>\n        </v-btn>\n      </div>\n    </div>\n    <v-row class=\"mt-5 pa-4 rounded-lg\" style=\"background-color: #a2af9b; color: black\">\n      <v-col\n        v-for=\"(node, index) in slave_nodes\"\n        :key=\"index\"\n        cols=\"12\"\n        sm=\"6\"\n        md=\"3\"\n      >\n        <v-sheet\n          class=\"pa-3 text-subtitle-1 text-blue-grey-darken-4 rounded-lg\"\n          elevation=\"1\"\n          color=\"primary\"\n        >\n          <v-sheet color=\"primary\">\n            <div class=\"d-flex justify-space-between align-center\">\n              <strong>STATUS :\n                <span :class=\"node.node_status === 'online' ? 'text-teal text-uppercase' : 'text-red-darken-2 text-uppercase'\">\n                  {{ node.node_status }}\n                </span>\n              </strong>\n              <div>\n                <v-btn\n                  icon=\"mdi-cog\"\n                  size=\"small\"\n                  variant=\"text\"\n                  @click.stop=\"showNodePopup(node)\"\n                  :disabled=\"node.node_status === 'offline'\"\n                ></v-btn>\n                <v-btn\n                  icon=\"mdi-delete\"\n                  size=\"small\"\n                  variant=\"text\"\n                  color=\"red-darken-2\"\n                  @click.stop=\"unpairNode(node)\"\n                  :disabled=\"node.node_status === 'offline'\"\n                ></v-btn>\n              </div>\n            </div>\n            <div class=\"d-flex align-center\">\n              <span class=\"mr-2\">Node Name :</span>\n              <span\n                v-if=\"editingNodeMac !== node.mac_address\"\n                @click.stop=\"startEditing(node)\"\n                style=\"cursor: pointer;\"\n                class=\"font-weight-bold\"\n              >\n                {{ node.name || 'Mesh Node' }}\n              </span>\n              <v-text-field\n                v-else\n                v-model=\"editingNodeName\"\n                @keyup.enter=\"saveNodeName\"\n                @blur=\"saveNodeName\"\n                @click.stop\n                dense\n                autofocus\n                single-line\n                hide-details\n              ></v-text-field>\n            </div>\n            <p>MAC Addr : {{ node.mac_address }}</p>\n            <p>Node ID : {{ node.node_id }}</p>\n            <p>Node Type : {{ node.node_type }}</p>\n            <p>Description : {{ node.description }}</p>\n            \n            <div v-if=\"node.node_type === 'sensor' && node.state && node.state.i2c && node.node_status == 'online'\" class=\"mt-2\">\n              <v-divider class=\"my-1\"></v-divider>\n              <p v-for=\"(value, key) in node.state.i2c\" :key=\"key\" class=\"mb-0 text-caption\">\n                <strong class=\"text-capitalize font-weight-bold\">{{ key }}:</strong> {{ value }}\n              </p>\n              <v-divider class=\"mt-1\"></v-divider>\n            </div>\n            <div v-if=\"node.node_type === 'relay' && node.node_status === 'online'\">\n              \n              <v-btn\n                class=\"mb-2 mt-2\"\n                block\n                color=\"blue-grey-darken-1\"\n                @click=\"setManualMode(node)\"\n              >\n                {{ node.manual_mode ? 'AUTO' : 'MANUAL' }}\n              </v-btn>\n              <v-divider class=\"my-2\"></v-divider>\n              <div class=\"d-flex justify-space-around\">\n                <v-switch\n                  v-model=\"node.state.r1\"\n                  label=\"Relay 1\"\n                  color=\"teal\"\n                  inset\n                  hide-details\n                  :disabled=\"!node.manual_mode\"\n                  @change.stop=\"toggleRelay(node, 'r1')\"\n                ></v-switch>\n                <v-switch\n                  v-model=\"node.state.r2\"\n                  label=\"Relay 2\"\n                  color=\"teal\"\n                  inset\n                  hide-details\n                  :disabled=\"!node.manual_mode\"\n                  @change.stop=\"toggleRelay(node, 'r2')\"\n                ></v-switch>\n              </div>\n            </div>\n          </v-sheet>\n        </v-sheet>\n      </v-col>\n    </v-row>\n    <v-dialog v-model=\"dialog\" max-width=\"600\">\n      <v-card v-if=\"selectedNode\">\n        <div class=\"text-h6 text-center text-blue-grey-darken-4 pa-3\" style=\"background-color: #a2af9b;\">\n          {{ selectedNode.name || 'Mesh Node' }}\n        </div>\n        \n        <v-list v-if=\"selectedNode.node_type === 'sensor' && editableNodeConfig.state && editableNodeConfig.state.gpio\">\n          <v-list-item \n            v-for=\"(gpioConfig, gpioPin) in editableNodeConfig.state.gpio\" \n            :key=\"gpioPin\"\n          >\n            <div class=\"d-flex flex-column flex-sm-row justify-space-between align-center\">\n              <div class=\"font-weight-bold mb-2 mb-sm-0\">\n                {{ gpioPin }}\n              </div>\n              <v-btn-toggle\n                v-model=\"gpio_config.mode[gpioPin]\"\n                variant=\"outlined\"\n                divided\n                density=\"compact\"\n              >\n                <v-btn \n                  value=\"button\" \n                  @click=\"updateGpioConfig(gpioPin, 'button')\" \n                >Button</v-btn>\n                <v-btn \n                  value=\"toggle\" \n                  @click=\"updateGpioConfig(gpioPin, 'toggle')\" \n                >Toggle</v-btn>\n                <v-btn \n                  value=\"delay\" \n                  @click=\"updateGpioConfig(gpioPin, 'delay')\" \n                >Delay</v-btn>\n              </v-btn-toggle>\n            </div>\n          </v-list-item>\n        </v-list>\n        <v-list v-else-if=\"selectedNode.node_type === 'relay'\">\n          <v-list-item>\n            <div class=\"d-flex flex-column flex-sm-row flex-wrap align-start align-sm-center\">\n              <strong class=\"mr-sm-4 mb-2 mb-sm-0\">Relay 1:</strong>\n              <div style=\"width: 100%;\" class=\"d-flex flex-column flex-sm-row flex-wrap align-start align-sm-center\">\n                <v-select\n                  v-model=\"editableNodeConfig.state.r1_sensor\"\n                  :items=\"sensorNodeList\"\n                  label=\"Sensor Node\"\n                  dense outlined hide-details\n                  class=\"mr-sm-2 mb-2 mb-sm-0 w-100 w-sm-auto\"\n                  style=\"max-width: 150px;\"\n                ></v-select>\n                <v-select\n                  v-model=\"editableNodeConfig.state.r1_sensor_type\"\n                  :items=\"['gpio', 'i2c']\"\n                  label=\"Type\"\n                  dense outlined hide-details\n                  class=\"mr-sm-2 mb-2 mb-sm-0 w-100 w-sm-auto\"\n                  style=\"max-width: 100px;\"\n                ></v-select>\n              </div>\n              <div class=\"d-flex flex-column\" style=\"width: 100%;\">\n                <span class=\"text-caption mb-1\" :class=\"{ 'text-disabled': !editableNodeConfig.state.r1_sensor_type }\">Pin/Addr:</span>\n                <v-btn-toggle\n                  v-model=\"editableNodeConfig.state.r1_sensor_pin\"\n                  variant=\"outlined\"\n                  density=\"compact\"\n                  divided\n                  :disabled=\"!editableNodeConfig.state.r1_sensor_type\"\n                  class=\"flex-wrap\"\n                >\n                  <v-btn\n                    v-for=\"pin in r1_sensor_pins\"\n                    :key=\"pin\"\n                    :value=\"pin\"\n                    class=\"flex-grow-1\"\n                  >\n                    {{ pin }}\n                  </v-btn>\n                </v-btn-toggle>\n              </div>\n              <div class=\"d-flex flex-column\" style=\"width: 100%;\">\n                <v-text-field\n                  v-if=\"editableNodeConfig.state.r1_sensor && editableNodeConfig.state.r1_sensor_type && editableNodeConfig.state.r1_sensor_type == 'i2c' && editableNodeConfig.state.r1_sensor_pin\"\n                  v-model=\"editableNodeConfig.state.r1_condition_name\"\n                  label=\"Work Condition Ex: >30\"\n                  dense\n                  outlined\n                  hide-details\n                  class=\"mt-4\"\n                ></v-text-field>\n                <v-btn-toggle\n                  v-if=\"editableNodeConfig.state.r1_sensor && editableNodeConfig.state.r1_sensor_type && editableNodeConfig.state.r1_sensor_type == 'gpio' && editableNodeConfig.state.r1_sensor_pin\"\n                  v-model=\"editableNodeConfig.state.r1_condition_name\"\n                  variant=\"outlined\"\n                  density=\"compact\"\n                  divided\n                  class=\"mt-4\"\n                >\n                  <v-btn :value=\"true\">TRUE</v-btn>\n                  <v-btn :value=\"false\">FAlSE</v-btn>\n                </v-btn-toggle>\n              </div>\n            </div>\n          </v-list-item>\n          <v-divider></v-divider>\n          <v-list-item>\n            <div class=\"d-flex flex-column flex-sm-row flex-wrap align-start align-sm-center\">\n              <strong class=\"mr-sm-4 mb-2 mb-sm-0\">Relay 2:</strong>\n              <div style=\"width: 100%;\" class=\"d-flex flex-column flex-sm-row flex-wrap align-start align-sm-center\">\n                <v-select\n                  v-model=\"editableNodeConfig.state.r2_sensor\"\n                  :items=\"sensorNodeList\"\n                  label=\"Sensor Node\"\n                  dense outlined hide-details\n                  class=\"mr-sm-2 mb-2 mb-sm-0 w-100 w-sm-auto\"\n                  style=\"max-width: 150px;\"\n                ></v-select>\n                <v-select\n                  v-model=\"editableNodeConfig.state.r2_sensor_type\"\n                  :items=\"['gpio', 'i2c']\"\n                  label=\"Type\"\n                  dense outlined hide-details\n                  class=\"mr-sm-2 mb-2 mb-sm-0 w-100 w-sm-auto\"\n                  style=\"max-width: 100px;\"\n                ></v-select>\n              </div>\n              <div class=\"d-flex flex-column\" style=\"width: 100%;\">\n                <span class=\"text-caption mb-1\" :class=\"{ 'text-disabled': !editableNodeConfig.state.r2_sensor_type }\">Pin/Addr:</span>\n                <v-btn-toggle\n                  v-model=\"editableNodeConfig.state.r2_sensor_pin\"\n                  variant=\"outlined\"\n                  density=\"compact\"\n                  divided\n                  :disabled=\"!editableNodeConfig.state.r2_sensor_type\"\n                  class=\"flex-wrap\"\n                >\n                  <v-btn\n                    v-for=\"pin in r2_sensor_pins\"\n                    :key=\"pin\"\n                    :value=\"pin\"\n                    class=\"flex-grow-1\"\n                  >\n                    {{ pin }}\n                  </v-btn>\n                </v-btn-toggle>\n              </div>\n              <div class=\"d-flex flex-column\" style=\"width: 100%;\">\n                <v-text-field\n                  v-if=\"editableNodeConfig.state.r2_sensor && editableNodeConfig.state.r2_sensor_type && editableNodeConfig.state.r2_sensor_type == 'i2c' && editableNodeConfig.state.r2_sensor_pin\"\n                  v-model=\"editableNodeConfig.state.r2_condition_name\"\n                  label=\"Work Condition Ex: >30\"\n                  dense\n                  outlined\n                  hide-details\n                  class=\"mt-4\"\n                ></v-text-field>\n                <v-btn-toggle\n                  v-if=\"editableNodeConfig.state.r2_sensor && editableNodeConfig.state.r2_sensor_type && editableNodeConfig.state.r2_sensor_type == 'gpio' && editableNodeConfig.state.r2_sensor_pin\"\n                  v-model=\"editableNodeConfig.state.r2_condition_name\"\n                  variant=\"outlined\"\n                  density=\"compact\"\n                  divided\n                  class=\"mt-4\"\n                >\n                  <v-btn :value=\"true\">TRUE</v-btn>\n                  <v-btn :value=\"false\">FAlSE</v-btn>\n                </v-btn-toggle>\n              </div>\n            </div>\n          </v-list-item>\n        </v-list>\n        \n        <v-card-actions class=\"pa-3\">\n          <v-btn\n            v-if=\"selectedNode.node_type === 'sensor'\"\n            style=\"background-color: #a2af9b; color: black\"\n            text\n            @click=\"saveNodeConfig\"\n          >\n            Save\n          </v-btn>\n          <v-btn\n            v-if=\"selectedNode.node_type === 'relay' && !selectedNode.config\"\n            style=\"background-color: #a2af9b; color: black\"\n            text\n            @click=\"saveNodeConfig\"\n          >\n            Save\n          </v-btn>\n          <v-btn\n            v-if=\"selectedNode.node_type === 'relay' && selectedNode.config\"\n            style=\"background-color: rgb(211, 59, 59); color: white\"\n            text\n            @click=\"deleteConfig(selectedNode)\"\n          >\n            Delete\n          </v-btn>\n          <v-spacer></v-spacer>\n          <v-btn \n            style=\"background-color: #90a4ae; color: black\" \n            text\n            @click=\"closeDialog\"\n          >\n            Close\n          </v-btn>\n        </v-card-actions>\n      </v-card>\n    </v-dialog>\n    <v-snackbar\n      v-model=\"snackbar\"\n      top\n      absolute\n      :timeout=\"notify_timeout\"\n      :color=\"notify_color\"\n      elevation=\"5\"\n    >\n      {{ message }}\n      <template v-slot:actions>\n        <v-btn text @click=\"snackbar = false\">Close</v-btn>\n      </template>\n    </v-snackbar>\n  </div>\n</template>\n<script>\nexport default {\n  name: \"NodeManager\",\n  mounted() {\n    if (localStorage.getItem(\"loggedIn\") === \"false\") {\n      this.redirectLogin();\n    }\n    this.getNodesMongoDB();\n    this.checkNodeStatus(); \n  },\n  data() {\n    return {\n      slave_nodes: [],\n      getNode_request: false,\n      ping_delay: 0,\n      allNode_request: false,\n      activeNode_request: false,\n      unpair_request: false,\n      gpioConfig_request: false,\n      getGpioConfig_request: false,\n      relayConfig_request: false,\n      getRelayConfig_request: false,\n      removeConfig_request: false,\n      toAuto_request: false,\n      toManual_request: false,\n      isLoading: false,\n      master_status: \"offline\",\n      editingNodeMac: null,\n      editingNodeName: \"\",\n      dialog: false,\n      selectedNode: null,\n      editableNodeConfig: null, // Temporary storage for edits in the dialog\n      snackbar: false,\n      message: '',\n      notify_color: \"teal\",\n      notify_timeout: 0,\n      gpio_config: {\n        node_id: \"\",\n        mode: {}\n      },\n    };\n  },\n  computed: {\n    // Returns a list of online sensor nodes for the dropdowns\n    sensorNodeList() {\n      return this.slave_nodes\n        .filter(n => n.node_type === 'sensor' && n.node_status === 'online')\n        .map(n => n.name || n.mac_address);\n    },\n    // Dynamically gets the available pins for the Relay 1 sensor\n    r1_sensor_pins() {\n      if (!this.editableNodeConfig || !this.editableNodeConfig.state.r1_sensor || !this.editableNodeConfig.state.r1_sensor_type) {\n        return [];\n      }\n      \n      const selectedNodeName = this.editableNodeConfig.state.r1_sensor;\n      const node = this.slave_nodes.find(n => (n.name === selectedNodeName || n.mac_address === selectedNodeName));\n      \n      if (!node || !node.state) return [];\n      const type = this.editableNodeConfig.state.r1_sensor_type;\n      if (type === 'gpio' && node.state.gpio) {\n        return Object.keys(node.state.gpio);\n      }\n      if (type === 'i2c' && node.state.i2c) { // Assuming i2c data might exist at node.state.i2c\n        return Object.keys(node.state.i2c);\n      }\n      \n      return [];\n    },\n    // Dynamically gets the available pins for the Relay 2 sensor\n    r2_sensor_pins() {\n      if (!this.editableNodeConfig || !this.editableNodeConfig.state.r2_sensor || !this.editableNodeConfig.state.r2_sensor_type) {\n        return [];\n      }\n      \n      const selectedNodeName = this.editableNodeConfig.state.r2_sensor;\n      const node = this.slave_nodes.find(n => (n.name === selectedNodeName || n.mac_address === selectedNodeName));\n      \n      if (!node || !node.state) return [];\n      const type = this.editableNodeConfig.state.r2_sensor_type;\n      if (type === 'gpio' && node.state.gpio) {\n        return Object.keys(node.state.gpio);\n      }\n      if (type === 'i2c' && node.state.i2c) {\n        return Object.keys(node.state.i2c);\n      }\n      \n      return [];\n    }\n  },\n  watch: {\n    msg(newMsg) {\n      if (newMsg.type == \"MONGO_DB\" && this.getNode_request) {\n        this.isLoading = false;\n        this.slave_nodes = Object.values(newMsg.payload).map(node => ({\n          ...node,\n          node_status: \"offline\",\n          manual_mode: false,\n        }));\n        this.getNode_request = false;\n      }\n      if (newMsg.type == \"PONG\") {\n        this.master_status = \"online\";\n        if (this.ping_delay < 2) {\n          this.ping_delay += 2;\n        }\n      }\n      if (newMsg.type == \"ACTIVE_NODE\" && this.activeNode_request) {\n        this.activeNode_request = false;\n        this.slave_nodes = this.slave_nodes.map(node => {\n          const key = node.mac_address;\n          if (newMsg.payload[key] && newMsg.payload[key].state) {\n            if (node.manual_mode == false) {\n              return {\n                ...node,\n                state: newMsg.payload[key]?.state || {},\n                node_status: \"online\"\n              };\n            } else {\n              return {\n                ...node,\n                node_status: \"online\"\n              };\n            }\n          } else {\n            return {\n              ...node,\n              node_status: \"offline\"\n            };\n          }\n        });\n      }\n      if (newMsg.type == \"ALL_NODE\" && this.allNode_request) {\n        this.allNode_request = false;\n        const dropObj = {\n          collection: \"nodes\",\n          payload: {},\n          operation: \"deleteMany\",\n          type: \"MONGO_DB\"\n        };\n        this.send(dropObj);\n        setTimeout(() => {\n          const insertObj = {\n            collection: \"nodes\",\n            payload: [Object.values(newMsg.payload)],\n            operation: \"insertMany\",\n            type: \"MONGO_DB\"\n          };\n          this.send(insertObj);\n          this.getNodesMongoDB();\n          this.showNotification(`Sync succeeded`, \"green\");\n        }, 1500);\n      }\n      \n      if (newMsg.type == \"UNPAIR_NODE\" && this.unpair_request) {\n        const deleteObj = {\n          collection: \"nodes\",\n          operation: \"deleteOne\",\n          type: \"MONGO_DB\",\n          payload: [{ mac_address: newMsg.payload.mac_address }]\n        };\n        this.send(deleteObj);\n        this.slave_nodes = this.slave_nodes.filter(\n          (n) => n.mac_address !== newMsg.payload.mac_address\n        );\n        this.unpair_request = false;\n        this.showNotification(`Unpair ${newMsg.payload.mac_address} succeeded`, \"green\");\n      }\n      if (newMsg.type == \"UPDATE_GPIO_CONFIG\" && this.gpioConfig_request) {\n        this.gpioConfig_request = false;\n        this.showNotification(`node_id : ${newMsg.payload.node_id} GPIO configuration updated successfully`, \"green\");\n      }\n      if (newMsg.type == \"GET_GPIO_CONFIG\" && this.getGpioConfig_request) {\n        this.getGpioConfig_request = false;\n        if (newMsg.payload && newMsg.payload.mode) {\n          this.gpio_config = {\n            node_id: newMsg.payload.node_id,\n            mode: newMsg.payload.mode\n          };\n        } else {\n          this.gpio_config = {\n            node_id: \"\",\n            mode: {}\n          };\n        }\n      }\n      if (newMsg.type == \"RELAY_CONFIG\" && this.relayConfig_request) {\n        this.relayConfig_request = false;\n        this.showNotification(`node_id : ${newMsg.payload.node_id} Relay configuration updated successfully`, \"green\");\n        this.updateConfigRelay();\n      }\n      if (newMsg.type == \"GET_RELAY_CONFIG\" && this.getRelayConfig_request) {\n        this.getRelayConfig_request = false;\n        const updatedRelays = Object.keys(newMsg.payload || {});\n        this.slave_nodes = this.slave_nodes.map(node => {\n          if (node.node_type === 'relay') {\n            const updatedRelay = updatedRelays.find(r => r == node.node_id);\n            if (updatedRelay) {\n              return {\n                ...node,\n                config: {\n                  [node.node_id]: newMsg.payload[updatedRelay]\n                }\n              };\n            }\n          }\n          return node;\n        });\n        // drop all and insert again\n        this.allNode_request = false;\n        const dropObj = {\n          collection: \"nodes\",\n          payload: {},\n          operation: \"deleteMany\",\n          type: \"MONGO_DB\"\n        };\n        this.send(dropObj);\n        \n        // add all node again\n        setTimeout(() => {\n          const insertObj = {\n            collection: \"nodes\",\n            payload: [Object.values(this.slave_nodes)],\n            operation: \"insertMany\",\n            type: \"MONGO_DB\"\n          };\n          this.send(insertObj);\n          this.getNodesMongoDB();\n          this.showNotification(`Sync succeeded`, \"green\");\n        }, 1500);\n      }\n      if (newMsg.type == \"REMOVE_CONFIG\" && this.removeConfig_request) {\n        this.removeConfig_request = false;\n        this.showNotification(`node_id : ${newMsg.payload.node_id} configuration removed successfully`, \"green\");\n        this.updateConfigRelay();\n      }\n      if (newMsg.type == \"REMOVE_CONFIG\" && this.toManual_request) {\n        this.toManual_request = false;\n        // change auto mode to manual mode\n        this.slave_nodes = this.slave_nodes.map(node => {\n          if (node.node_id === newMsg.payload.node_id) {\n            return {\n              ...node,\n              manual_mode: true\n            };\n          }\n          return node;\n        });\n      }\n      if (newMsg.type == \"RELAY_CONFIG\" && this.toAuto_request) {\n        this.toAuto_request = false;\n        // change manual mode to auto mode\n        this.slave_nodes = this.slave_nodes.map(node => {\n          if (node.node_id === newMsg.payload.node_id) {\n            return {\n              ...node,\n              manual_mode: false\n            };\n          }\n          return node;\n        }); \n      }\n      if (newMsg.type == \"GET_GPIO_CONFIG\" && this.getGpioConfig_request) {\n        this.snackbar = false;\n    }\n  },\n  methods: {\n    getGpioConfig() {\n      if (this.master_status !== \"online\") {\n        this.showNotification(\"Master node is offline. Cannot fetch GPIO configurations.\", \"red\");\n        return;\n      }\n      const configObj = {\n        payload: \"getConfig\",\n        type: \"GET_GPIO_CONFIG\",\n      };\n      this.getGpioConfig_request = true;\n      this.send(configObj);\n    },\n    updateConfigRelay() {\n      if (this.master_status !== \"online\") {\n        this.showNotification(\"Master node is offline. Cannot fetch relay configurations.\", \"red\");\n        return;\n      }\n      const configObj = {\n        payload: \"getConfig\",\n        type: \"GET_RELAY_CONFIG\",\n      };\n      this.getRelayConfig_request = true;\n      this.send(configObj);\n    },\n    saveNodeConfig() {\n      if (!this.selectedNode) return;\n      if (this.selectedNode.node_type === 'sensor') {\n        this.saveSensorConfig();\n      } else if (this.selectedNode.node_type === 'relay') {\n        this.saveRelayConfig();\n      }\n    },\n    saveRelayConfig() {\n      // map editableNodeConfig r1_sensor to node_id\n      if (!this.editableNodeConfig) return;\n      const r1_sensor_node = this.slave_nodes.find(n => (n.name === this.editableNodeConfig.state.r1_sensor || n.mac_address === this.editableNodeConfig.state.r1_sensor));\n      const r2_sensor_node = this.slave_nodes.find(n => (n.name === this.editableNodeConfig.state.r2_sensor || n.mac_address === this.editableNodeConfig.state.r2_sensor));\n      const configObj = {\n        payload: {\n          [this.editableNodeConfig.node_id]: {\n            r1: {\n              node_id: r1_sensor_node ? r1_sensor_node.node_id : null,\n              condition: `${this.editableNodeConfig.state.r1_sensor_pin}|${this.editableNodeConfig.state.r1_condition_name}` || null,\n            },\n            r2: {\n              node_id: r2_sensor_node ? r2_sensor_node.node_id : null,\n              condition: `${this.editableNodeConfig.state.r2_sensor_pin}${this.editableNodeConfig.state.r2_condition_name}` || null,\n            }\n          }\n        },\n        type: \"RELAY_CONFIG\",\n      };\n      this.send(configObj);\n      this.showNotification(\"Saving relay configuration...\", \"blue\");\n      this.relayConfig_request = true;\n      this.closeDialog();\n    },\n    saveSensorConfig() {\n      const updateObj = {\n        payload: this.gpio_config,\n        type: \"UPDATE_GPIO_CONFIG\",\n      };\n      this.send(updateObj);\n      this.gpioConfig_request = true;\n      this.showNotification(\"Saving GPIO configuration...\", \"blue\");\n      this.closeDialog();\n    },\n    closeDialog() {\n      this.dialog = false;\n      this.gpio_config = {\n        node_id: \"\",\n        mode: {}\n      };\n      this.selectedNode = null;\n      this.editableNodeConfig = null;\n    },\n    updateGpioConfig(gpioPin, newType) {\n      this.gpio_config.mode[gpioPin] = newType;\n    },\n    showNotification(msg = 'Notification shown at the top!', color = \"teal\") {\n      this.notify_timeout += 3000;\n      this.message = msg;\n      this.notify_color = color;\n      this.snackbar = true;\n      this.$emit('send', { payload: 'top_notification_shown' });\n    },\n    unpairNode(node) {\n      if (this.master_status !== \"online\") {\n        this.showNotification(\"Master node is offline. Cannot fetch relay configurations.\", \"red\");\n        return;\n      }\n      if (node.node_status == \"online\") {\n        if (confirm(`Are you sure you want to unpair the node \"${node.name || node.mac_address}\"?`)) {\n          const unpairObj = {\n            payload: node.node_id,\n            type: \"UNPAIR_NODE\",\n          }\n          this.send(unpairObj);\n          this.unpair_request = true;\n          this.showNotification(\"Unpairing...\", \"blue\");\n        }\n      } else {\n        this.showNotification(`This ${node.mac_address} is offline`, \"red\");\n      }\n    },\n    setManualMode(node) {\n      if (!node.manual_mode) {\n        const removeObj = {\n          payload: node.node_id,\n          type: \"REMOVE_CONFIG\",\n        }\n        this.send(removeObj);\n        this.toManual_request = true;\n        this.showNotification(`Toggle ${node.name || node.mac_address} to manual....`, \"blue\");\n      } else {\n        if (node.config) {\n          const configObj = {\n            payload: node.config,\n            type: \"RELAY_CONFIG\",\n          };\n          this.send(configObj);\n          this.toAuto_request = true;\n          this.showNotification(`Toggle ${node.name || node.mac_address} to auto....`, \"blue\");\n        } else {\n          this.showNotification(\"No configuration found for this relay node. Please set up the configuration first.\", \"red\");\n          return;\n        }\n      }\n    },\n    toggleRelay(node) {\n      if (!node || !node.state) {\n        return;\n      }\n      const commandObj = {\n        type: \"RELAY_CONTROL\",\n        payload: {\n          node_id: node.node_id,\n          state: {\n            \"r1\": node.state.r1,\n            \"r2\": node.state.r2,\n          },\n        }\n      };\n      this.send(commandObj);\n    },\n    showNodePopup(node) {\n      if (master_status)\n      if (node.node_type == \"sensor\") {\n        let default_gpio = node.state.gpio || {};\n        for (let key in default_gpio) {\n          default_gpio[key] = \"button\";\n        } \n        this.gpio_config = {\n          node_id: node.node_id,\n          mode: default_gpio || {}\n        };\n        this.getGpioConfig();\n      }\n      this.selectedNode = node;\n      this.editableNodeConfig = JSON.parse(JSON.stringify(node));\n      this.dialog = true;\n    },\n    startEditing(node) {\n      this.editingNodeMac = node.mac_address;\n      this.editingNodeName = node.name || '';\n    },\n    saveNodeName() {\n      if (!this.editingNodeMac) return;\n      const nodeToUpdate = this.slave_nodes.find(n => n.mac_address === this.editingNodeMac);\n      if (nodeToUpdate) {\n        nodeToUpdate.name = this.editingNodeName;\n        const updateObj = {\n          collection: \"nodes\",\n          operation: \"updateOne\",\n          type: \"MONGO_DB\",\n          payload: [\n            { mac_address: this.editingNodeMac },\n            { $set: { name: this.editingNodeName } }\n          ]\n        };\n        this.send(updateObj);\n      }\n      this.editingNodeMac = null;\n      this.editingNodeName = \"\";\n    },\n    redirectLogin() {\n      const base = window.location.origin;\n      window.location.href = `${base}/dashboard/login`;\n    },\n    getNodesMongoDB() {\n      const sendObj = {\n        collection: \"nodes\",\n        operation: \"find.toArray\",\n        type: \"MONGO_DB\"\n      };\n      this.getNode_request = true;\n      this.send(sendObj);\n    },\n    checkNodeStatus() {\n      const sendObj = { payload: \"active\", type: \"ACTIVE_NODE\" };\n      setInterval(() => {\n        if (this.master_status == \"online\") {\n          this.activeNode_request = true;\n          this.send(sendObj);\n        }\n      }, 10000);\n    },\n    syncSlave() {\n      if (this.master_status !== \"online\") {\n        this.showNotification(\"Master node is offline. Cannot fetch relay configurations.\", \"red\");\n        return;\n      }\n      this.allNode_request = true;\n      const sendObj = { payload: \"getAllNode\", type: \"ALL_NODE\" };\n      this.send(sendObj);\n      this.isLoading = true;\n    },\n    refreshPage() {\n      window.location.reload();\n    },\n    removeConfig(node) {\n      const removeObj = {\n        payload: node.node_id,\n        type: \"REMOVE_CONFIG\",\n      }\n      this.send(removeObj);\n      this.removeConfig_request = true;\n    },\n    deleteConfig(node) {\n      if (confirm(`Are you sure you want to delete the configuration for the node \"${node.name || node.mac_address}\"?`)) {\n        this.removeConfig(node);\n        this.showNotification(\"Removing configuration...\", \"blue\");\n        this.closeDialog();\n      }\n    },\n  }\n}\n}\n</script>\n<style scoped>\n/* Add any component-specific styles here */\n</style>\n\n<!-- \n\n- get gpio config\n- get relay config on get active payload\n\n-->",
        "storeOutMessages": true,
        "passthru": true,
        "resendOnRefresh": true,
        "templateScope": "local",
        "className": "",
        "x": 2280,
        "y": 960,
        "wires": [
            [
                "e0e44357057ba5c3"
            ]
        ],
        "inputLabels": [
            "tyesty"
        ]
    },
    {
        "id": "414b2c245a73e2f4",
        "type": "mongodb3 in",
        "z": "2c682173eb064de0",
        "service": "_ext_",
        "configNode": "013c016d2aaac063",
        "name": "mongoDB",
        "collection": "",
        "operation": "",
        "x": 1720,
        "y": 2060,
        "wires": [
            [
                "b17e40686d04010d"
            ]
        ]
    },
    {
        "id": "60f63e5e64f49467",
        "type": "mongodb3 in",
        "z": "2c682173eb064de0",
        "service": "_ext_",
        "configNode": "013c016d2aaac063",
        "name": "mongoDB",
        "collection": "",
        "operation": "",
        "x": 2300,
        "y": 1920,
        "wires": [
            [
                "1e4b8208b3d66eb8"
            ]
        ]
    },
    {
        "id": "1d0bd63eacdc4dfc",
        "type": "inject",
        "z": "2c682173eb064de0",
        "name": "Pseudo Paired",
        "props": [
            {
                "p": "payload"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "{\"mac_address\":\"0C:AD:ER:BE:HU:15\",\"node_id\":1950750929,\"node_type\":\"sensor\",\"info\":\"pair success!\"}",
        "payloadType": "json",
        "x": 920,
        "y": 2260,
        "wires": [
            [
                "e3794c8ee1dd02c6"
            ]
        ]
    },
    {
        "id": "c68ea67af5dd77d5",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt in : res/mesh/pair",
        "info": "",
        "x": 910,
        "y": 2220,
        "wires": []
    },
    {
        "id": "e103019d912b0671",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Pseudo Active Node",
        "func": "if (msg.payload) {\n    msg.payload = {\n        \"A8:46:74:46:18:DC\": {\n            \"mac_address\": \"A8:46:74:46:18:DC\",\n            \"node_id\": 1950750941,\n            \"node_type\": \"relay\",\n            \"state\": {\n            \"r1\": true,\n            \"r2\": true\n            }\n        },\n        \"A8:46:74:46:18:D0\": {\n            \"mac_address\": \"A8:46:74:46:18:D0\",\n            \"node_id\": 1950750929,\n            \"node_type\": \"sensor\",\n            \"state\": {\n                \"gpio\" : {\n                    \"gpio_0\": false,\n                    \"gpio_1\": false,\n                    \"gpio_3\": false\n                },\n                \"i2c\" : {\n                    \"temperature\": 33.9,\n                    \"humudity\": 53.2,\n                    \"light\": 26.66667\n                }\n            }\n        }\n    }\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2280,
        "y": 1660,
        "wires": [
            [
                "e05016ef1c99d9ab"
            ]
        ]
    },
    {
        "id": "14b63b61051ac714",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Active Node Req",
        "func": "if (msg.type == \"ACTIVE_NODE\") {\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2040,
        "y": 1660,
        "wires": [
            [
                "e103019d912b0671"
            ]
        ]
    },
    {
        "id": "e05016ef1c99d9ab",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Active Node Res",
        "func": "msg.type = \"ACTIVE_NODE\";\nreturn msg;",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2540,
        "y": 1660,
        "wires": [
            [
                "b9bb8e0b80aefb9d"
            ]
        ]
    },
    {
        "id": "a5a7bd0418d69e9a",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Pseudo All Node",
        "func": "if (msg.payload) {\n    msg.payload = {\n        \"A8:46:74:46:18:DC\": {\n            \"mac_address\": \"A8:46:74:46:18:DC\",\n            \"node_id\": 1950750941,\n            \"node_type\": \"relay\"\n        },\n        \"A8:46:74:46:18:D0\": {\n            \"mac_address\": \"A8:46:74:46:18:D0\",\n            \"node_id\": 1950750929,\n            \"node_type\": \"sensor\"\n        }\n    }\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2270,
        "y": 580,
        "wires": [
            [
                "52d435fc7da18a62"
            ]
        ]
    },
    {
        "id": "07d88fd0251f9976",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter All Node Req",
        "func": "if (msg.type == \"ALL_NODE\") {\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2030,
        "y": 580,
        "wires": [
            [
                "a5a7bd0418d69e9a"
            ]
        ]
    },
    {
        "id": "52d435fc7da18a62",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter All Node Res",
        "func": "msg.type = \"ALL_NODE\";\nreturn msg;",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2510,
        "y": 580,
        "wires": [
            [
                "fd7426e6c315f693"
            ]
        ]
    },
    {
        "id": "859b3bd66bc892ae",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Req Mongo",
        "func": "if (msg.type == \"MONGO_DB\") {\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2150,
        "y": 1800,
        "wires": [
            [
                "60f63e5e64f49467"
            ]
        ]
    },
    {
        "id": "1e4b8208b3d66eb8",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Res Mongo",
        "func": "if (!msg.payload.result) {\n    msg.type = \"MONGO_DB\";\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2450,
        "y": 1800,
        "wires": [
            [
                "c02fc4796c3e4414"
            ]
        ]
    },
    {
        "id": "db79adae9b08456e",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Mongo",
        "func": "if (msg.toMongo){\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 930,
        "y": 1640,
        "wires": [
            [
                "90f6a692f0f33d60"
            ]
        ]
    },
    {
        "id": "90f6a692f0f33d60",
        "type": "mongodb3 in",
        "z": "2c682173eb064de0",
        "service": "_ext_",
        "configNode": "013c016d2aaac063",
        "name": "mongoDB",
        "collection": "",
        "operation": "",
        "x": 1100,
        "y": 1640,
        "wires": [
            []
        ]
    },
    {
        "id": "55a623171fc79f7d",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt out : req/mesh/allnodes",
        "info": "",
        "x": 2140,
        "y": 540,
        "wires": []
    },
    {
        "id": "d3363709dc27bd68",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt in : res/mesh/allnodes",
        "info": "",
        "x": 2390,
        "y": 540,
        "wires": []
    },
    {
        "id": "3529f7b1daa289a4",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt out : req/mesh/nodes",
        "info": "",
        "x": 2150,
        "y": 1700,
        "wires": []
    },
    {
        "id": "62b7f6335b217409",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt in: res/mesh/nodes",
        "info": "",
        "x": 2420,
        "y": 1700,
        "wires": []
    },
    {
        "id": "d19c169f0e8a8e3a",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt out : esp/ping",
        "info": "",
        "x": 2170,
        "y": 260,
        "wires": []
    },
    {
        "id": "d05016c063f7ce6a",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt in : esp/pong",
        "info": "",
        "x": 2370,
        "y": 260,
        "wires": []
    },
    {
        "id": "940767fa7b87149e",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Pseudo Ping Pong",
        "func": "if (msg.payload) {\n    msg.payload = \"alive\"\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2270,
        "y": 300,
        "wires": [
            [
                "a8ef2b924362df80"
            ]
        ]
    },
    {
        "id": "a8ef2b924362df80",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Pong",
        "func": "msg.type = \"PONG\"\nreturn msg;",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2390,
        "y": 440,
        "wires": [
            [
                "12e519c8df58a2c6"
            ]
        ]
    },
    {
        "id": "7edb7302290b1cde",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Pseudo Relay Control",
        "func": "return msg;",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2280,
        "y": 700,
        "wires": [
            [
                "360c61bb6dc9d5a6"
            ]
        ]
    },
    {
        "id": "f8761ba5befd1d2f",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt out : req/state/update",
        "info": "",
        "x": 2290,
        "y": 660,
        "wires": []
    },
    {
        "id": "d2410c09cb80c845",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Relay Control",
        "func": "if (msg.type == \"RELAY_CONTROL\") {\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2050,
        "y": 700,
        "wires": [
            [
                "7edb7302290b1cde"
            ]
        ]
    },
    {
        "id": "360c61bb6dc9d5a6",
        "type": "debug",
        "z": "2c682173eb064de0",
        "name": "Relay Debug",
        "active": false,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "payload",
        "targetType": "msg",
        "statusVal": "",
        "statusType": "auto",
        "x": 2510,
        "y": 700,
        "wires": []
    },
    {
        "id": "702e35ad5726d375",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt out : req/mesh/unpair",
        "info": "",
        "x": 2110,
        "y": 1560,
        "wires": []
    },
    {
        "id": "42bca23738056e66",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Pseudo Unpair Node",
        "func": "if (msg.payload) {\n  msg.payload = {\n    \"node_id\": 12345,\n    \"mac_address\": \"A8:46:74:46:18:DC\",\n    \"info\": \"unpairing\"\n  }\n  return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2240,
        "y": 1520,
        "wires": [
            [
                "f40a20bbd027a35c"
            ]
        ]
    },
    {
        "id": "139f4630319af6dc",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Unpair Node Req",
        "func": "if (msg.type == \"UNPAIR_NODE\") {\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2020,
        "y": 1520,
        "wires": [
            [
                "42bca23738056e66"
            ]
        ]
    },
    {
        "id": "5f2ecbfb9a7e4650",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt in : res/mesh/unpair",
        "info": "",
        "x": 2390,
        "y": 1560,
        "wires": []
    },
    {
        "id": "f40a20bbd027a35c",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Unpair Node Res",
        "func": "msg.type = \"UNPAIR_NODE\";\nreturn msg;",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2480,
        "y": 1520,
        "wires": [
            [
                "9a1992c72110a3a2"
            ]
        ]
    },
    {
        "id": "4888745f99eb533c",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt out : esp/ping",
        "info": "",
        "x": 890,
        "y": 1360,
        "wires": []
    },
    {
        "id": "66f1bc45562a3b3e",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt in : esp/pong",
        "info": "",
        "x": 1090,
        "y": 1360,
        "wires": []
    },
    {
        "id": "819491c8890ba0e6",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Pseudu Update GPIO Type",
        "func": "if (msg.payload) {\n    msg.payload = { \n        \"node_id\": 1950750929, \n        \"info\": \"OK\" \n    }\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2240,
        "y": 1400,
        "wires": [
            [
                "c480f62ecfe42887"
            ]
        ]
    },
    {
        "id": "3e5dd7ea9e590980",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Update GPIO Type",
        "func": "if (msg.type == \"UPDATE_GPIO_CONFIG\") {\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 1950,
        "y": 1400,
        "wires": [
            [
                "819491c8890ba0e6"
            ]
        ]
    },
    {
        "id": "c480f62ecfe42887",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Update GPIO Type",
        "func": "msg.type = \"UPDATE_GPIO_CONFIG\";\nreturn msg;",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2510,
        "y": 1400,
        "wires": [
            [
                "bc0df7681d9cbc45"
            ]
        ]
    },
    {
        "id": "944e6a18b5fe141d",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt out : req/mode/update",
        "info": "",
        "x": 2110,
        "y": 1440,
        "wires": []
    },
    {
        "id": "254aeb489644d27f",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt in : res/mode/update",
        "info": "",
        "x": 2390,
        "y": 1440,
        "wires": []
    },
    {
        "id": "4b136e59afc1504c",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Pseudu Config Relay",
        "func": "if (msg.payload) {\n    msg.payload = { \n        \"node_id\": 1950750941, \n        \"status\": \"OK\", \"info\": \"OK\" \n    }\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2260,
        "y": 820,
        "wires": [
            [
                "65efb4c29d5c46cb"
            ]
        ]
    },
    {
        "id": "d36963b9ce8297ee",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Config Relay",
        "func": "if (msg.type == \"RELAY_CONFIG\") {\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 1970,
        "y": 820,
        "wires": [
            [
                "4b136e59afc1504c"
            ]
        ]
    },
    {
        "id": "65efb4c29d5c46cb",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Config Relay",
        "func": "msg.type = \"RELAY_CONFIG\";\nreturn msg;",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2530,
        "y": 820,
        "wires": [
            [
                "0828b628b6f651dc"
            ]
        ]
    },
    {
        "id": "e25703f56feee270",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt out : req/config/set",
        "info": "",
        "x": 2140,
        "y": 780,
        "wires": []
    },
    {
        "id": "ad5c3f8c01c32084",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt in : res/config/set",
        "info": "",
        "x": 2420,
        "y": 780,
        "wires": []
    },
    {
        "id": "ae729bab3af5c2c1",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Pseudu Get Config",
        "func": "if (msg.payload) {\n    msg.payload = {\n        \"1950750941\": {\n            \"r1\": {\n                \"node_id\":1950750929,\n                \"condition\":\"gpio_1|true\"\n            },\n            \"r2\": {\n                \"node_id\":1950750929,\n                \"condition\":\"light>200\"\n            }\n        },\n        \"195075230941\": {\n            \"r1\": {\n                \"node_id\": 1950750929,\n                \"condition\": \"gpio_1|true\"\n            },\n            \"r2\": {\n                \"node_id\": 1950750929,\n                \"condition\": \"light>200\"\n            }\n        }\n    }\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2250,
        "y": 1100,
        "wires": [
            [
                "70eddee43adda882"
            ]
        ]
    },
    {
        "id": "6797372e78d6ff0d",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Get Config",
        "func": "if (msg.type == \"GET_RELAY_CONFIG\") {\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2000,
        "y": 1100,
        "wires": [
            [
                "ae729bab3af5c2c1"
            ]
        ]
    },
    {
        "id": "70eddee43adda882",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Get Config",
        "func": "msg.type = \"GET_RELAY_CONFIG\";\nreturn msg;",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2500,
        "y": 1100,
        "wires": [
            [
                "077ac4e53213c43f"
            ]
        ]
    },
    {
        "id": "72e14380c85e165f",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt out : req/config/remove",
        "info": "",
        "x": 2120,
        "y": 1140,
        "wires": []
    },
    {
        "id": "4deaf2be35e97e2f",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt in : res/config/remove",
        "info": "",
        "x": 2390,
        "y": 1140,
        "wires": []
    },
    {
        "id": "7a8a346844e20fb9",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Pseudu Remove Config",
        "func": "if (msg.payload) {\n    msg.payload = {\n        \"node_id\": 1950750941,\n        \"status\": \"OK\",\n        \"info\": \"OK\"\n    }\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2270,
        "y": 1240,
        "wires": [
            [
                "57af227c25c7db9a"
            ]
        ]
    },
    {
        "id": "3250d2b2f41803d5",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Remove Config",
        "func": "if (msg.type == \"REMOVE_CONFIG\") {\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2020,
        "y": 1240,
        "wires": [
            [
                "7a8a346844e20fb9"
            ]
        ]
    },
    {
        "id": "57af227c25c7db9a",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Remove Config",
        "func": "msg.type = \"REMOVE_CONFIG\";\nreturn msg;",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2520,
        "y": 1240,
        "wires": [
            [
                "01f7625f3734e1db"
            ]
        ]
    },
    {
        "id": "18964b086de651ae",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt out : req/config/get",
        "info": "",
        "x": 2140,
        "y": 1280,
        "wires": []
    },
    {
        "id": "d4e655e627178cf5",
        "type": "comment",
        "z": "2c682173eb064de0",
        "name": "mqtt in : res/config/get",
        "info": "",
        "x": 2380,
        "y": 1280,
        "wires": []
    },
    {
        "id": "67e152f122d9f6a8",
        "type": "inject",
        "z": "2c682173eb064de0",
        "name": "",
        "props": [
            {
                "p": "payload"
            }
        ],
        "repeat": "10",
        "crontab": "",
        "once": true,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "ping",
        "payloadType": "str",
        "x": 790,
        "y": 1400,
        "wires": [
            [
                "50d414316f279371"
            ]
        ]
    },
    {
        "id": "d773be3b7865caa0",
        "type": "inject",
        "z": "2c682173eb064de0",
        "name": "",
        "props": [
            {
                "p": "payload"
            }
        ],
        "repeat": "10",
        "crontab": "",
        "once": true,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "ping",
        "payloadType": "str",
        "x": 2010,
        "y": 300,
        "wires": [
            [
                "940767fa7b87149e"
            ]
        ]
    },
    {
        "id": "e5cae6e1a5e75d0f",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Get GPIO Config",
        "func": "if (msg.type == \"REMOVE_CONFIG\") {\n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2020,
        "y": 1040,
        "wires": [
            [
                "b64e8c08b3a0ad44"
            ]
        ]
    },
    {
        "id": "5111cd9cd6961a11",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Filter Get GPIO Config",
        "func": "msg.type = \"GET_GPIO_CONFIG\";\nreturn msg;",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2520,
        "y": 1040,
        "wires": [
            [
                "077ac4e53213c43f"
            ]
        ]
    },
    {
        "id": "b64e8c08b3a0ad44",
        "type": "function",
        "z": "2c682173eb064de0",
        "name": "Pseudu Get GPIO Config",
        "func": "if (msg.payload) {\n    \n    return msg;\n}",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 2270,
        "y": 1040,
        "wires": [
            [
                "5111cd9cd6961a11"
            ]
        ]
    },
    {
        "id": "4d73b1b5c177d13a",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 480,
        "y": 1840,
        "wires": [
            [
                "4472b5190b4b72f8",
                "65f08f8689f4b1ee"
            ]
        ]
    },
    {
        "id": "567f64296f314b8a",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 1560,
        "y": 1760,
        "wires": [
            [
                "f3bda6c28c4cdefd"
            ]
        ]
    },
    {
        "id": "ca478c1c869a4381",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 1480,
        "y": 1540,
        "wires": [
            [
                "567f64296f314b8a"
            ]
        ]
    },
    {
        "id": "2662d247e5832ef4",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 640,
        "y": 1540,
        "wires": [
            []
        ]
    },
    {
        "id": "4472b5190b4b72f8",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 560,
        "y": 1980,
        "wires": [
            [
                "628aaeee6693ac2b",
                "b0e27f5ab60e12fd"
            ]
        ]
    },
    {
        "id": "b0e27f5ab60e12fd",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 640,
        "y": 2220,
        "wires": [
            [
                "dadc1fd165490ce2"
            ]
        ]
    },
    {
        "id": "990288c0b07710ec",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 1480,
        "y": 1980,
        "wires": [
            [
                "567f64296f314b8a"
            ]
        ]
    },
    {
        "id": "8c89bbddc8ff1946",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 1420,
        "y": 2420,
        "wires": [
            [
                "990288c0b07710ec"
            ]
        ]
    },
    {
        "id": "dadc1fd165490ce2",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 720,
        "y": 2420,
        "wires": [
            [
                "cb049ac75fb69645"
            ]
        ]
    },
    {
        "id": "65f08f8689f4b1ee",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 560,
        "y": 1640,
        "wires": [
            [
                "db79adae9b08456e"
            ]
        ]
    },
    {
        "id": "e0e44357057ba5c3",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 1300,
        "y": 1000,
        "wires": [
            [
                "36f53e223aef6872",
                "5159011c320839ba"
            ]
        ]
    },
    {
        "id": "f4a13cbca9ec8ca7",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 3200,
        "y": 920,
        "wires": [
            [
                "81d74a33ea5da797"
            ]
        ]
    },
    {
        "id": "34af37b65b55eacf",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 1800,
        "y": 1800,
        "wires": [
            [
                "859b3bd66bc892ae"
            ]
        ]
    },
    {
        "id": "c02fc4796c3e4414",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 2760,
        "y": 1800,
        "wires": [
            [
                "b9bb8e0b80aefb9d"
            ]
        ]
    },
    {
        "id": "dc3ea815b143fda4",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 1180,
        "y": 1540,
        "wires": [
            []
        ]
    },
    {
        "id": "f7f4e351ed6248e3",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 1540,
        "y": 580,
        "wires": [
            [
                "07d88fd0251f9976"
            ]
        ]
    },
    {
        "id": "c14b30fed8c2b15d",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 1720,
        "y": 1660,
        "wires": [
            [
                "34af37b65b55eacf",
                "14b63b61051ac714"
            ]
        ]
    },
    {
        "id": "fd7426e6c315f693",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 3020,
        "y": 580,
        "wires": [
            [
                "0828b628b6f651dc"
            ]
        ]
    },
    {
        "id": "b9bb8e0b80aefb9d",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 2840,
        "y": 1660,
        "wires": [
            [
                "9a1992c72110a3a2"
            ]
        ]
    },
    {
        "id": "12e519c8df58a2c6",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 2940,
        "y": 440,
        "wires": [
            [
                "fd7426e6c315f693"
            ]
        ]
    },
    {
        "id": "ae7c5b3d7e4a5795",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 1460,
        "y": 700,
        "wires": [
            [
                "f7f4e351ed6248e3",
                "d2410c09cb80c845"
            ]
        ]
    },
    {
        "id": "2350652fbadaae99",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 1640,
        "y": 1520,
        "wires": [
            [
                "c14b30fed8c2b15d",
                "139f4630319af6dc"
            ]
        ]
    },
    {
        "id": "9a1992c72110a3a2",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 2920,
        "y": 1520,
        "wires": [
            [
                "bc0df7681d9cbc45"
            ]
        ]
    },
    {
        "id": "bc0df7681d9cbc45",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 3000,
        "y": 1400,
        "wires": [
            [
                "01f7625f3734e1db"
            ]
        ]
    },
    {
        "id": "498c72312f925ecb",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 1560,
        "y": 1400,
        "wires": [
            [
                "2350652fbadaae99",
                "3e5dd7ea9e590980"
            ]
        ]
    },
    {
        "id": "36f53e223aef6872",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 1380,
        "y": 820,
        "wires": [
            [
                "ae7c5b3d7e4a5795",
                "d36963b9ce8297ee"
            ]
        ]
    },
    {
        "id": "0828b628b6f651dc",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 3100,
        "y": 820,
        "wires": [
            [
                "f4a13cbca9ec8ca7"
            ]
        ]
    },
    {
        "id": "5159011c320839ba",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 1380,
        "y": 1100,
        "wires": [
            [
                "6797372e78d6ff0d",
                "24838956d182cb22",
                "e5cae6e1a5e75d0f"
            ]
        ]
    },
    {
        "id": "077ac4e53213c43f",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 3120,
        "y": 1100,
        "wires": [
            [
                "f4a13cbca9ec8ca7"
            ]
        ]
    },
    {
        "id": "24838956d182cb22",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 1480,
        "y": 1240,
        "wires": [
            [
                "498c72312f925ecb",
                "3250d2b2f41803d5"
            ]
        ]
    },
    {
        "id": "01f7625f3734e1db",
        "type": "junction",
        "z": "2c682173eb064de0",
        "x": 3080,
        "y": 1240,
        "wires": [
            [
                "077ac4e53213c43f"
            ]
        ]
    },
    {
        "id": "2f692e7cb40097ed",
        "type": "ui-group",
        "name": "Login Group",
        "page": "f721a59bf36641c2",
        "width": 12,
        "height": 1,
        "order": 1,
        "showTitle": false,
        "className": "",
        "visible": false,
        "disabled": "false",
        "groupType": "default"
    },
    {
        "id": "8146a75c4508b858",
        "type": "ui-group",
        "name": "Scan Group",
        "page": "b5fbde398eca86ab",
        "width": 12,
        "height": 1,
        "order": 1,
        "showTitle": false,
        "className": "",
        "visible": "true",
        "disabled": "false",
        "groupType": "default"
    },
    {
        "id": "dd337cb0663d76c3",
        "type": "ui-group",
        "name": "Home",
        "page": "b820a1f0025766bd",
        "width": 12,
        "height": 1,
        "order": 1,
        "showTitle": false,
        "className": "",
        "visible": "true",
        "disabled": "false",
        "groupType": "default"
    },
    {
        "id": "013c016d2aaac063",
        "type": "mongodb3",
        "uri": "mongodb+srv://tuwaris:5D4Wyu40hIK64EFh@tuwaris.5ynbx.mongodb.net/smart_home?retryWrites=true&w=majority&appName=Tuwaris",
        "name": "smart_home",
        "options": "{ “useNewUrlParser”: true}",
        "parallelism": -1
    },
    {
        "id": "f721a59bf36641c2",
        "type": "ui-page",
        "name": "Login",
        "ui": "9c6816c5eb8f8852",
        "path": "/login",
        "icon": "login",
        "layout": "grid",
        "theme": "6ba0f05d9b3e74ec",
        "breakpoints": [
            {
                "name": "Default",
                "px": "0",
                "cols": "3"
            },
            {
                "name": "Tablet",
                "px": "576",
                "cols": "6"
            },
            {
                "name": "Small Desktop",
                "px": "768",
                "cols": "9"
            },
            {
                "name": "Desktop",
                "px": "1024",
                "cols": "12"
            }
        ],
        "order": 1,
        "className": "",
        "visible": true,
        "disabled": false
    },
    {
        "id": "b5fbde398eca86ab",
        "type": "ui-page",
        "name": "Scan",
        "ui": "9c6816c5eb8f8852",
        "path": "/scan",
        "icon": "wifi",
        "layout": "grid",
        "theme": "6ba0f05d9b3e74ec",
        "breakpoints": [
            {
                "name": "Default",
                "px": "0",
                "cols": "3"
            },
            {
                "name": "Tablet",
                "px": "576",
                "cols": "6"
            },
            {
                "name": "Small Desktop",
                "px": "768",
                "cols": "9"
            },
            {
                "name": "Desktop",
                "px": "1024",
                "cols": "12"
            }
        ],
        "order": 2,
        "className": "",
        "visible": true,
        "disabled": false
    },
    {
        "id": "b820a1f0025766bd",
        "type": "ui-page",
        "name": "Home",
        "ui": "9c6816c5eb8f8852",
        "path": "/home",
        "icon": "home",
        "layout": "grid",
        "theme": "6ba0f05d9b3e74ec",
        "breakpoints": [
            {
                "name": "Default",
                "px": "0",
                "cols": "3"
            },
            {
                "name": "Tablet",
                "px": "576",
                "cols": "6"
            },
            {
                "name": "Small Desktop",
                "px": "768",
                "cols": "9"
            },
            {
                "name": "Desktop",
                "px": "1024",
                "cols": "12"
            }
        ],
        "order": 3,
        "className": "",
        "visible": "true",
        "disabled": "false"
    },
    {
        "id": "9c6816c5eb8f8852",
        "type": "ui-base",
        "name": "My Dashboard",
        "path": "/dashboard",
        "appIcon": "",
        "includeClientData": true,
        "acceptsClientConfig": [
            "ui-notification",
            "ui-control"
        ],
        "showPathInSidebar": false,
        "headerContent": "page",
        "navigationStyle": "default",
        "titleBarStyle": "default",
        "showReconnectNotification": true,
        "notificationDisplayTime": 1,
        "showDisconnectNotification": true,
        "allowInstall": true
    },
    {
        "id": "6ba0f05d9b3e74ec",
        "type": "ui-theme",
        "name": "Default Theme",
        "colors": {
            "surface": "#a2af9b",
            "primary": "#dccfc0",
            "bgPage": "#faf9ee",
            "groupBg": "#eeeeee",
            "groupOutline": "#e3e3e3"
        },
        "sizes": {
            "density": "default",
            "pagePadding": "12px",
            "groupGap": "12px",
            "groupBorderRadius": "4px",
            "widgetGap": "12px"
        }
    },
    {
        "id": "9f4aa0f4364dd65a",
        "type": "global-config",
        "env": [],
        "modules": {
            "@flowfuse/node-red-dashboard": "1.27.2",
            "node-red-contrib-mongodb3": "2.0.1"
        }
    }
]