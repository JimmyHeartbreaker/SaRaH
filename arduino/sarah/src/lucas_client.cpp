#include "../headers/lucas_client.h"
#include "mbed.h"
#include "WiFiInterface.h"
#include "TCPSocket.h"
// Global WiFi client

// Reconnection timing
static unsigned long lastConnectAttempt = 0;
static const unsigned long CONNECT_RETRY_MS = 5000;


     TCPSocket base_socket;
    SocketAddress server;
/* ============================================================
   Public API
   ============================================================ */
void setup_lucas_client() {
  Serial.println();
  Serial.println("[lucas_client] Starting...");

  connectToWiFi();

  lastConnectAttempt = 0;
}


int get_bytes(uint8_t* buf, size_t buf_size) 
{   
    return base_socket.recvfrom (&server, buf, buf_size);
}


bool send_bytes(const uint8_t *data, size_t len, unsigned long timeoutMs) {
  

  size_t sent = 0;
  unsigned long start = millis();

  const size_t CHUNK_SIZE = 512; // try 512 or less
  while (sent < len) { 

    size_t to_send = min((size_t)CHUNK_SIZE, len - sent);
    size_t written = base_socket.sendto(server,data + sent, to_send);
      
    if (written <= 0 ) {
      if (millis() - start > timeoutMs) {
        Serial.println("[wifi] write stalled, aborting");
        base_socket.close();
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
int connectToWiFi() {
  
 // Serial.println("Wi-Fi SocketWrapper example\n");

    WiFiInterface *wifi = WiFiInterface::get_default_instance();
    nsapi_connection_status_t status = wifi->get_connection_status();
    if(status != NSAPI_STATUS_DISCONNECTED )
    { 
   //   Serial.println("already connected");
      return 0;
    }

    if (!wifi) {
        Serial.println("ERROR: No WiFiInterface found.\n");
        return -1;
    }
    
    Serial.println("Connecting to Wi-Fi...\n");
     nsapi_error_t ret ;
    do
    {
      ret = wifi->connect(LUCAS_WIFI_SSID,LUCAS_PASSWORD, NSAPI_SECURITY_WPA_WPA2 );
     if (ret != 0) {
        Serial.println("Wi-Fi connection failed\n");
        delay(1000);
    }
      /* code */
    } while (ret!=0);
    
    
    SocketAddress address;
    wifi->get_ip_address(&address);
    Serial.println("Connected!");// IP address: %s\n", );
    Serial.println(address.get_ip_address());
    return 0;
}
// void disconnectFromServer()
// {
//     WiFiInterface *wifi = WiFiInterface::get_default_instance();

// }
int tryConnectToServer() {

  Serial.println("tryConnectToServer");
    WiFiInterface *wifi = WiFiInterface::get_default_instance();
    nsapi_error_t ret = wifi->gethostbyname(LUCAS_HOST_IP, &server);
    if (ret != NSAPI_ERROR_OK) {
        Serial.println("DNS lookup failed:");
        wifi->disconnect();
        return -1;
    }

    server.set_port(LUCAS_PORT);
   // printf("Resolved server:", server.get_ip_address(), server.get_port());

    base_socket.open(wifi);

    // Retry connection
    for (int attempts = 0; attempts < 5; attempts++) {
        ret = base_socket.connect(server);
        if (ret == NSAPI_ERROR_OK) {
            Serial.println("Connection success!");
            break;
        } else {
            Serial.println("Connection failed");
               delay(1000);
        }
    }
    
  
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
