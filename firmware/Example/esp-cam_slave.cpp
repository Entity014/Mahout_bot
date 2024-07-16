#include <Arduino.h>
#include <Wire.h>
#include <ESP32QRCodeReader.h>

#define I2C_DEV_ADDR 0x55

ESP32QRCodeReader reader(CAMERA_MODEL_AI_THINKER);

String payload = "";

void onRequest();
void onReceive(int len);
void onQrCodeTask(void *pvParameters);

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  reader.setup();
  Serial.println("Setup QRCode Reader");
  reader.beginOnCore(1);
  Serial.println("Begin on Core 1");
  xTaskCreate(onQrCodeTask, "onQrCode", 4 * 1024, NULL, 4, NULL);
}

void loop()
{
}

void onQrCodeTask(void *pvParameters)
{
  struct QRCodeData qrCodeData;

  Wire.setPins(14, 15);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
  while (true)
  {
    if (reader.receiveQrCode(&qrCodeData, 100))
    {
      Serial.println("Found QRCode");
      if (qrCodeData.valid)
      {
        Serial.print("Payload: ");
        Serial.println((const char *)qrCodeData.payload);
        payload = (const char *)qrCodeData.payload;
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

void onRequest()
{
  Wire.print(payload);
  payload = "";
  Serial.println("onRequest");
  Serial.println();
}

void onReceive(int len)
{
  Serial.printf("onReceive[%d]: ", len);
  while (Wire.available())
  {
    Serial.write(Wire.read());
  }
  Serial.println();
}
