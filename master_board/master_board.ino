#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_BMP085.h>
#include "HX711.h"
#include <time.h>

// BMP180 setup
Adafruit_BMP085 bmp;

// HX711 load cell setup
#define DOUT 23
#define CLK 19
HX711 scale(DOUT, CLK);
float calibration_factor = 211000;

// ESP-NOW setup: MAC address of the slave ESP32
uint8_t slaveMAC[] = {0x24, 0x0A, 0xC4, 0x9B, 0xD6, 0xC4}; 
// Example MAC address, use Serial.println(WiFi.macAddress());
// to find MAC address of slave board

// Structure to hold the data to send
typedef struct struct_message {
  float temperature;
  float pressure;
  float load;
  unsigned long timestamp;
} struct_message;

// Create a message object
struct_message dataToSend;

// Callback when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\nLast Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Function to initialize sensors
void initSensors() {
  // Initialize BMP180
  if (!bmp.begin()) {
    Serial.println("Could not find BMP180 sensor!");
    while (1) {}
  }

  // Initialize HX711
  scale.set_scale();
  scale.tare(); // Reset the scale to 0

  // Initialize time for timestamp (NTP or manual setup)
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("Waiting for NTP time sync...");
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
  } else {
    Serial.println("Time synchronized");
  }
}

// Function to get sensor data
void getSensorData() {
  // Get temperature and pressure from BMP180
  dataToSend.temperature = bmp.readTemperature();
  dataToSend.pressure = bmp.readPressure() / 100.0F; // Convert Pa to hPa (millibars)

  // Get load from HX711 load cell
  scale.set_scale(calibration_factor);
  dataToSend.load = scale.get_units(5);
  if (dataToSend.load < 0) {
    dataToSend.load = 0.0;
  }

  // Get timestamp
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    dataToSend.timestamp = mktime(&timeinfo);
  } else {
    dataToSend.timestamp = 0;
  }
}

// Function to send data via ESP-NOW
void sendData() {
  esp_err_t result = esp_now_send(slaveMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));

  // Check for success
  if (result == ESP_OK) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Error sending data");
  }

  // Print to serial monitor
  Serial.print("Temperature: ");
  Serial.print(dataToSend.temperature);
  Serial.print(" °C, Pressure: ");
  Serial.print(dataToSend.pressure);
  Serial.print(" hPa, load: ");
  Serial.print(dataToSend.load);
  Serial.print(" kg, Timestamp: ");
  Serial.println(dataToSend.timestamp);
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set the ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register send callback
  esp_now_register_send_cb(onDataSent);

  // Add slave peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, slaveMAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Initialize sensors
  initSensors();
}

void loop() {
  // Get sensor data
  getSensorData();

  // Send data via ESP-NOW
  sendData();

  // Wait 5 seconds before next transmission
  delay(5000);
}
