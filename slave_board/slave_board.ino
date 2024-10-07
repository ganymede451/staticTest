#include <esp_now.h>
#include <WiFi.h>
#include <time.h>

typedef struct struct_message {
  float temperature;
  float pressure;    
  float load;       
  unsigned long timestamp; 
} struct_message;

// Create a message object to hold the incoming data
struct_message incomingData;

// Callback when data is received
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingDataBytes, int len) {
  // Copy the received data into the incomingData object
  memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));

  // Convert timestamp to human-readable format
  time_t t = incomingData.timestamp;
  struct tm *tm_info = localtime(&t);
  char timeString[25];
  strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", tm_info);

  // Convert load from kg to newtons
  float loadInNewtons = incomingData.load * 9.81;

  // Log the data
  Serial.print("Received data from: ");
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Temperature: ");
  Serial.println(incomingData.temperature);
  Serial.print("Pressure: ");
  Serial.println(incomingData.pressure);
  Serial.print("Load: ");
  Serial.println(loadInNewtons); // Print load in newtons
  Serial.print("Timestamp: ");
  Serial.println(timeString);
  Serial.println();

}

void setup() {
  Serial.begin(115200);
  // Sets the ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
  // Nothing to do in the loop, data is received via callback
}
