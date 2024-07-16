#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "config.h"
#include "motor.h"
#include "pid.h"

#define I2C_DEV_ADDR 0x55
#define initial_speed 800
#define rotation_speed 900

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);

PID motor_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Servo servo_right;
Servo servo_left;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

int state = 0;
int pre_btn = 0;
bool isFour = false;

imu::Vector<3> euler_temp, euler;

bool isStartX = false;
bool isStartY = false;
bool isStartZ = false;

uint16_t lastPosition = 0;
const uint8_t SensorCount = 8;
int sensorValues[SensorCount];

void onReceive(int len);
void buttonISR();
uint16_t readLine(int *sensorValues, int sensorCount);
void onIMUTask(void *pvParameters);
void onCameraTask(void *pvParameters);

void setup()
{
  Serial.begin(115200);
  Serial.println();
  pinMode(QTR1, INPUT);
  pinMode(QTR2, INPUT);
  pinMode(QTR3, INPUT);
  pinMode(QTR4, INPUT);
  pinMode(QTR5, INPUT);
  pinMode(QTR6, INPUT);
  pinMode(QTR7, INPUT);
  pinMode(QTR8, INPUT);

  pinMode(BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN), buttonISR, CHANGE);

  servo_right.attach(SERVO_RIGHT);
  servo_left.attach(SERVO_LEFT);
  servo_right.write(90);
  servo_left.write(180);

  Wire.begin((uint8_t)I2C_DEV_ADDR);
  Wire.onReceive(onReceive);
  Serial.println("Setup I2C Communication");

  // if (!bno.begin())
  // {
  //   /* There was a problem detecting the BNO055 ... check your connections */
  //   Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //   while (1)
  //     ;
  // }
  // bno.setExtCrystalUse(true);

  // xTaskCreatePinnedToCore(
  //     onIMUTask,
  //     "onIMU",
  //     4 * 1024,
  //     NULL,
  //     0,
  //     NULL,
  //     0);
  // xTaskCreatePinnedToCore(
  //     onCameraTask,
  //     "onCamera",
  //     4 * 1024,
  //     NULL,
  //     1,
  //     NULL,
  //     1);
}

void loop()
{
  sensorValues[0] = (analogRead(QTR1) >= 4095) ? 1 : 0;
  sensorValues[1] = (analogRead(QTR2) >= 4095) ? 1 : 0;
  sensorValues[2] = (analogRead(QTR3) >= 4095) ? 1 : 0;
  sensorValues[3] = (analogRead(QTR4) >= 4095) ? 1 : 0;
  sensorValues[4] = (analogRead(QTR5) >= 4095) ? 1 : 0;
  sensorValues[5] = (analogRead(QTR6) >= 4095) ? 1 : 0;
  sensorValues[6] = (analogRead(QTR7) >= 4095) ? 1 : 0;
  sensorValues[7] = (analogRead(QTR8) >= 4095) ? 1 : 0;
  // Serial.printf("1: %u 2: %u 3: %u 4: %u 5: %u 6: %u 7: %u 8: %u\n",
  //               sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3],
  //               sensorValues[4], sensorValues[5], sensorValues[6], sensorValues[7]);
  uint16_t position = readLine(sensorValues, 8);
  // servo_right.write(170);
  // servo_left.write(100);

  imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  if ((euler_temp.x() != e.x()) && (!isStartX))
  {
    euler_temp.x() = e.x();
    isStartX = true;
  }
  if ((euler_temp.y() != e.y()) && (!isStartY))
  {
    euler_temp.y() = e.y();
    isStartY = true;
  }
  if ((euler_temp.z() != e.z()) && (!isStartZ))
  {
    euler_temp.z() = e.z();
    isStartZ = true;
  }
  euler.x() = euler_temp.x() - e.x();
  euler.y() = euler_temp.y() - e.y();
  euler.z() = euler_temp.z() - e.z();
  // Serial.printf("x: %f y: %f z: %f\n", e.x(), e.y(), e.z());

  if (state == 0)
  {
    motor1_controller.spin(0);
    motor2_controller.spin(0);
    motor3_controller.spin(0);
  }
  else if (state == 1)
  {
    if (position == 28 && !isFour)
    {
      motor1_controller.spin(initial_speed);
      motor2_controller.spin(initial_speed);
      motor3_controller.spin(0);
      delay(300);
      isFour = true;
    }
    else if (sensorValues[0])
    {
      motor1_controller.spin(rotation_speed);
      motor2_controller.spin(0);
      motor3_controller.spin(-rotation_speed);
    }
    else if (sensorValues[7])
    {
      motor1_controller.spin(0);
      motor2_controller.spin(rotation_speed);
      motor3_controller.spin(rotation_speed);
    }
    else if (sensorValues[1])
    {
      motor1_controller.spin(rotation_speed);
      motor2_controller.spin(initial_speed);
      motor3_controller.spin(0);
    }
    else if (sensorValues[6])
    {
      motor1_controller.spin(initial_speed);
      motor2_controller.spin(rotation_speed);
      motor3_controller.spin(0);
    }
    else
    {
      motor1_controller.spin(initial_speed);
      motor2_controller.spin(initial_speed);
      motor3_controller.spin(0);
    }
  }
  else
  {
    state = 0;
    lastPosition = 3;
    isFour = false;
  }
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

void buttonISR()
{
  if (pre_btn != !digitalRead(BTN))
  {
    if (!digitalRead(BTN))
    {
      state++;
    }
    pre_btn = !digitalRead(BTN);
  }
}

void onIMUTask(void *pvParameters)
{
  while (true)
  {
  }
}

void onCameraTask(void *pvParameters)
{
  while (true)
  {
  }
}

uint16_t readLine(int *sensorValues, int sensorCount)
{
  uint32_t avg = 0;
  uint16_t sum = 0;

  for (uint8_t i = 0; i < sensorCount; i++)
  {
    uint16_t value = sensorValues[i];
    avg += value * i;
    sum += value;
  }
  lastPosition = avg;
  return lastPosition;
}