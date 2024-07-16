#include <Arduino.h>
#include "Wire.h"
#include <ESP32QRCodeReader.h>
#include "config.h"

#define I2C_DEV_ADDR 0x55
ESP32QRCodeReader reader(CAMERA_MODEL_AI_THINKER);

uint32_t i = 0;

void onQrCodeTask(void *pvParameters);

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
}

void loop()
{
  delay(100);
}

void onQrCodeTask(void *pvParameters)
{
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("Setup I2C Communication");
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

        Wire.beginTransmission(I2C_DEV_ADDR);
        Wire.printf((const char *)qrCodeData.payload);
        uint8_t error = Wire.endTransmission(true);
        Serial.printf("endTransmission: %u\n", error);
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