#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID "066e3a20-8455-4bf0-ad16-1da23293f628"        // Replace with your desired service UUID
#define CHARACTERISTIC_UUID "6e05613f-6d0c-4418-895f-db12044ccff9" // Replace with your desired characteristic UUID

#define BUTTON_PIN 14
#define LED 2

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

bool pre_button = false;
bool buttonPressed = false;

void IRAM_ATTR handleInterrupt();

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

void setup()
{
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleInterrupt, CHANGE);

  BLEDevice::init("ESP32_BLE_Stop1");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);

  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

void loop()
{
  if (deviceConnected)
  {
    digitalWrite(LED, HIGH);
    if (pre_button != buttonPressed)
    {
      if (buttonPressed)
      {
        Serial.println("Stop1");
        String data = "Stop1";
        pCharacteristic->setValue(data.c_str());
        pCharacteristic->notify();
      }
      pre_button = buttonPressed;
    }
  }
}

void IRAM_ATTR handleInterrupt()
{
  buttonPressed = !(digitalRead(BUTTON_PIN)) ? true : false;
}