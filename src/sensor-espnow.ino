//
//  "Battery" Sensor Client - Brad Black (C) 2021
//  Headless version adapted to use ESPNOW protocol to send data to another ESP32 for routing/relaying to Adafruit IO
//
//  Features:
//      - Temp Sensor from Water tight sensor
//      - Deep Sleep to conserve power consumption
//      - Send data over ESPNOW

#include <WiFi.h>
#include <esp_sleep.h>
#include "esp_wifi.h"

#include <esp_now.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// #define OPERATION_MODE // No Serial Debug
#define TEST_MODE // Serial Debug on for TESTING ONLY
#define TEMPSENSORPIN 25 // Digital pin connected sensor

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
/* You can get this MAC address value with a simple sketch like this run on the receiver ESP32 module...
  void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
  }
*/

// Structure to send data

typedef struct msg
{
  char name[32];
  float temperature;
  float voltage;
} msg;

msg sensorData;

esp_now_peer_info_t peerInfo;

OneWire oneWire(TEMPSENSORPIN);
DallasTemperature sensors(&oneWire);

bool readSensors()
{
  sensors.requestTemperatures();
  delay(1000);
  sensorData.temperature = sensors.getTempCByIndex(0);

  if (isnan(sensorData.temperature))
  {
    Serial.println("Failed to read from temp sensor!");
    return false;
  }
  else
  {
    return true;
  }
}

void sleep_now()
{
  esp_wifi_stop();
  esp_sleep_enable_timer_wakeup(600e6);  // Deep Sleep for 10 minutes
  esp_deep_sleep_start();
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup()
{

#if defined(TEST_MODE)
  Serial.begin(115200);
  while (!Serial)
  {
  }
  Serial.println("\nstarted");
#endif

  esp_wifi_stop();
  delay(10);
  pinMode(TEMPSENSORPIN, INPUT_PULLUP);

  sensors.begin();

  // Set device as a Wi-Fi Station & set "Low Data Rate Mode for ESPNOW"
  WiFi.mode(WIFI_STA);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register Callback function to get status of transmitted data

  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

} // setup

void loop(void)
{

  if (readSensors())
  {

    Serial.print("sending temp-> ");
    Serial.println(sensorData.temperature);

    Serial.print("sending voltage-> ");
    Serial.println(sensorData.voltage);

    strcpy(sensorData.name, "Pool");

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sensorData, sizeof(sensorData));

    if (result == ESP_OK)
    {
      Serial.println("Sent with success");
    }
    else
    {
      Serial.println("Error sending the data");
    }
  }
  else
    Serial.println("failed to read sensor");

  delay(1000);
  WiFi.disconnect(true);
  delay(100);
  sleep_now();
}