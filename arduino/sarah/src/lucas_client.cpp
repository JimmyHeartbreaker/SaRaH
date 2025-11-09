#include "../headers/lucas_client.h"

// Global WiFi client
static WiFiClient client;

// Reconnection timing
static unsigned long lastConnectAttempt = 0;
static const unsigned long CONNECT_RETRY_MS = 5000;

// Forward declarations (internal)
static void tryConnectToServer();
static void connectToWiFi();

/* ============================================================
   Public API
   ============================================================ */
void setup_lucas_client() {
  Serial.println();
  Serial.println("[lucas_client] Starting...");

  connectToWiFi();
  lastConnectAttempt = 0;
}


void loop_lucas_client() {
  // Ensure WiFi stays connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[lucas_client] WiFi disconnected. Reconnecting...");
    WiFi.disconnect();
    connectToWiFi();
  }

  // Ensure TCP connection
  if (!client || !client.connected()) {
    tryConnectToServer();
  }

  // Read incoming TCP data
  if (client && client.connected() && client.available() > 0) {
    const size_t BUF_SZ = 256;
    uint8_t buf[BUF_SZ];
    size_t n = client.read(buf, BUF_SZ);
    if (n > 0) {
      handle_received_bytes(buf, n, client.remoteIP(), client.remotePort());
    }
  }

}


bool send_bytes(const uint8_t *data, size_t len, unsigned long timeoutMs) {
  size_t sent = 0;
  unsigned long start = millis();

  const size_t CHUNK_SIZE = 512; // try 512 or less
  while (sent < len) {
    // if (!client.connected()) {
    //   Serial.println("[wifi] disconnected during send");
    //   return false;
    // }

 

    size_t to_send = min((size_t)CHUNK_SIZE, len - sent);
    size_t written = client.write(data + sent, to_send);
    if (written == 0) {
      if (millis() - start > timeoutMs) {
        Serial.println("[wifi] write stalled, aborting");
        return false;
      }
      delay(5);
      continue;
    }

    sent += written;
    start = millis();  // reset timeout after progress
  }

  return true;
}


/* ============================================================
   Internal helpers
   ============================================================ */
static void connectToWiFi() {
  Serial.print("[lucas_client] Connecting to SSID: ");
  Serial.println(LUCAS_WIFI_SSID);

  WiFi.begin(LUCAS_WIFI_SSID,LUCAS_PASSWORD);  // open network (no password)
  unsigned long start = millis();
  const unsigned long WAIT_MS = 10000;

  while (WiFi.status() != WL_CONNECTED && (millis() - start) < WAIT_MS) {
    Serial.print('.');
    delay(500);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[lucas_client] WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[lucas_client] WiFi connection failed (will retry in loop).");
  }
}


static void tryConnectToServer() {
  unsigned long now = millis();
  if (now - lastConnectAttempt < CONNECT_RETRY_MS) return;
  lastConnectAttempt = now;

  Serial.print("[lucas_client] Connecting to TCP ");
  Serial.print(LUCAS_HOST_IP);
  Serial.print(':');
  Serial.println(LUCAS_PORT);

  if (!client.connect(LUCAS_HOST_IP, LUCAS_PORT)) {
    Serial.println("[lucas_client] TCP connect failed.");
    client.stop();
    return;
  }

  Serial.println("[lucas_client] TCP connected!");
  client.flush();
}


/* ============================================================
   Default handler (can be overridden)
   ============================================================ */
void handle_received_bytes(const uint8_t *buf, size_t len, IPAddress remoteIP, uint16_t remotePort) {
  Serial.print("[lucas_client] Data from ");
  Serial.print(remoteIP);
  Serial.print(':');
  Serial.print(remotePort);
  Serial.print(" -> ");

  bool printable = true;
  for (size_t i = 0; i < len; ++i) {
    if (buf[i] < 0x09 || (buf[i] > 0x0A && buf[i] < 0x20)) {
      printable = false;
      break;
    }
  }

  if (printable) {
    Serial.write(buf, len);
  } else {
    for (size_t i = 0; i < len; ++i) {
      if (buf[i] < 16) Serial.print('0');
      Serial.print(buf[i], HEX);
      Serial.print(' ');
    }
  }
  Serial.println();
}
