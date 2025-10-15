<template class="bg-black-100">
  <div class="flex flex-col items-center justify-center h-full p-6">
    <h2 class="text-2xl font-bold mb-4">Login</h2>
    <v-text-field v-model="username" label="Username" outlined></v-text-field>
    <v-text-field v-model="password" label="Password" type="password" outlined></v-text-field>
    <p v-if="error" class="text-red-500 mt-2">{{ error }}</p>
    <v-btn class="mt-4" color="primary" @click="login">Login</v-btn>
  </div>
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
      username: "",
      password: "",
      error: ""
    };
  },
  mounted() {
    if (localStorage.getItem("loggedIn") == "true") {
      this.redirectScan();
    }
  },
  watch: {
    msg(newMsg) {
      if (newMsg && newMsg.payload) {
        const user = newMsg.payload;
        if (this.username === user.username && this.password === user.password) {
          this.error = "";
          localStorage.setItem("loggedIn", "true");
          this.redirectScan();
        } else {
          this.error = "Invalid username or password";
        }
      }
    }
  },
  methods: {
    login() {
      const sendObj = {
        collection: "users",
        payload: { username: this.username },
        operation: "findOne"
      };
      this.send(sendObj);
    },
    redirectScan() {
      // tell Node-RED to switch tab via ui-control
      const base = window.location.origin;
      window.location.href = `${base}/dashboard/scan`;
    }
  }
};
</script>
