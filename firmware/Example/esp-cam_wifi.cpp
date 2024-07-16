#include <Arduino.h>
#include "Wire.h"
#include <ESP32QRCodeReader.h>
#include "config.h"
#include <esp_now.h>
#include <WiFi.h>

ESP32QRCodeReader reader(CAMERA_MODEL_AI_THINKER);
uint8_t broadcastAddress[] = {0xB8, 0xF0, 0x09, 0xB3, 0x40, 0xF4};

typedef struct struct_message
{
  int id; // must be unique for each sender board
  String payload;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

void onQrCodeTask(void *pvParameters);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  reader.setup();
  Serial.println("Setup QRCode Reader");

  reader.beginOnCore(1);
  Serial.println("Begin on Core 1");

  xTaskCreate(onQrCodeTask, "onQrCode", 4 * 1024, NULL, 4, NULL);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
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
}

void loop()
{
  delay(100);
}

void onQrCodeTask(void *pvParameters)
{
  struct QRCodeData qrCodeData;

  while (true)
  {
    if (reader.receiveQrCode(&qrCodeData, 100))
    {
      Serial.println("Found QRCode");

      if (qrCodeData.valid)
      {
        Serial.print("Payload: ");
        Serial.println((const char *)qrCodeData.payload);
        myData.id = 1;
        myData.payload = (const char *)qrCodeData.payload;

        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

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
      {
        Serial.print("Invalid: ");
        Serial.println((const char *)qrCodeData.payload);
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}