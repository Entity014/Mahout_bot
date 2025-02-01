#include <Arduino.h>
#include <WiFi.h>

#define BUTTON_PIN 14
#define LED 2

const char *ssid = "iloveaut";
const char *password = "autloveme";
const char *server_ip = "192.168.1.100"; // Replace with your PC's IP
const int server_port = 12345;

WiFiClient client;

bool pre_button = false;
bool buttonPressed = false;

void IRAM_ATTR handleInterrupt();

void setup()
{
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  pinMode(LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleInterrupt, CHANGE);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  if (client.connect(server_ip, server_port))
  {
    Serial.println("Connected to server");
    digitalWrite(LED, HIGH);
  }
  else
  {
    Serial.println("Connection failed");
  }
}

void loop()
{
  // Send data every 5 seconds
  if (client.connected())
  {
    if (pre_button != buttonPressed)
    {
      if (buttonPressed)
      {
        client.println("Stop1");
      }
      pre_button = buttonPressed;
    }
  }
  else
  {
    client.connect(server_ip, server_port);
  }
}

void IRAM_ATTR handleInterrupt()
{
  buttonPressed = !(digitalRead(BUTTON_PIN)) ? true : false;
}