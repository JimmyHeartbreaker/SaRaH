#ifndef WIFI_SOCKET_H
#define WIFI_SOCKET_H

#include <Arduino.h>

// ============================================================
// Configuration
// ============================================================

#define LUCAS_WIFI_SSID "Galaxy"//"VM0685951"// "Vodafone701713"         // open network
#define LUCAS_HOST_IP    "172.28.140.75"//"192.168.1.148"    // your PC IP
#define LUCAS_PORT        9000
#define LUCAS_PASSWORD "zyebbrtbHT7tyqc6" //  "zyebbrtbHT7tyqc6"

// ============================================================
// Public API
// ============================================================

// Initializes the WiFi and prepares the client
void setup_lucas_client();
int connectToWiFi();
int tryConnectToServer();
// Handles reconnections and reads incoming data
int get_bytes(uint8_t* buf, size_t buf_size);

// Sends bytes to the connected TCP server (blocking)
bool send_bytes(const uint8_t* data, size_t len, unsigned long timeoutMs = 1000);
void close();
// Callback when data arrives — override if needed
//void handle_received_bytes(const uint8_t *buf, size_t len, IPAddress remoteIP, uint16_t remotePort);

#endif  // WIFI_SOCKET_H
