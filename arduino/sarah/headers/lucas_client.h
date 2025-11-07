#ifndef LUCAS_CLIENT_H
#define LUCAS_CLIENT_H

#include <Arduino.h>
#include <WiFi.h>  // or WiFiS3.h depending on your board

// ============================================================
// Configuration
// ============================================================

#define LUCAS_WIFI_SSID  "Galaxy"         // open network
#define LUCAS_HOST_IP    "10.150.121.75"    // your PC IP
#define LUCAS_PORT        9000

// ============================================================
// Public API
// ============================================================

// Initializes the WiFi and prepares the client
void setup_lucas_client();

// Handles reconnections and reads incoming data
void loop_lucas_client();

// Sends bytes to the connected TCP server (blocking)
bool send_bytes(const uint8_t* data, size_t len);

// Callback when data arrives — override if needed
void handle_received_bytes(const uint8_t *buf, size_t len, IPAddress remoteIP, uint16_t remotePort);

#endif  // LUCAS_CLIENT_H
