#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

#define BUTTON_PIN 14
#define LED 2

bool pre_button = false;
bool buttonPressed = false;
void IRAM_ATTR handleInterrupt();

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT_Device3"); // Name of your ESP32 Bluetooth device
  pinMode(LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleInterrupt, CHANGE);
}

void loop()
{
  if (pre_button != buttonPressed)
  {
    if (buttonPressed)
    {
      SerialBT.println("3");
      Serial.println("3");
    }
    pre_button = buttonPressed;
  }
}

void IRAM_ATTR handleInterrupt()
{
  buttonPressed = !(digitalRead(BUTTON_PIN)) ? true : false;
}