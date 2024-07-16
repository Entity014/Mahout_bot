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
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);

PID motor_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

Servo servo_right;
Servo servo_left;

int tower_right = 8;
int tower_left = 7;
int state = 0, robot_state = 0;
int pre_btn = 0;
bool send_data = false;
uint8_t payload = 0;

uint16_t lastPosition = 0, position = 0;
const uint8_t SensorCount = 8;
int sensorValues[SensorCount];

imu::Vector<3> euler_temp, euler;

bool isStartX = false;
bool isStartY = false;
bool isStartZ = false;

float calculateDegree(float degree_in, float degree_out);
void buttonISR();
void onInputTask(void *pvParameters);
uint16_t readLine(int *sensorValues, int sensorCount);

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.begin();

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  bno.setExtCrystalUse(true);

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
}

void loop()
{
  // Serial.printf("position: %u\n", position);
  // Serial.printf("euler x: %f abs_diff: %f diff: %f\n", euler.x(), diff, euler.y());
  // Serial.printf("data: %i tower: %i %i\n", payload, tower_left, tower_right);
  // uint8_t system, gyro, accel, mag = 0;
  // Serial.print("CALIBRATION: Sys=");
  // Serial.print(system, DEC);
  // Serial.print(" Gyro=");
  // Serial.print(gyro, DEC);
  // Serial.print(" Accel=");
  // Serial.print(accel, DEC);
  // Serial.print(" Mag=");
  // Serial.println(mag, DEC);

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
  position = readLine(sensorValues, 8);

  imu::Vector<3>
      e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  if (send_data && state == 1)
  {
    Wire.beginTransmission(I2C_DEV_ADDR);
    Wire.printf("Hello World!");
    uint8_t error = Wire.endTransmission(true);
    uint8_t bytesReceived = Wire.requestFrom(I2C_DEV_ADDR, 16);
    if ((bool)bytesReceived)
    { // If received more than zero bytes
      uint8_t temp[bytesReceived];
      Wire.readBytes(temp, bytesReceived);
      log_print_buf(temp, bytesReceived);
      char charValue = char(temp[0]);
      payload = charValue - '0';
    }
    if (payload <= 5 && payload > 0 && robot_state == 0)
    {
      if ((euler_temp.x() != e.x()))
      {
        euler_temp.x() = e.x();
      }
      if ((euler_temp.y() != e.y()))
      {
        euler_temp.y() = e.y();
      }
      if ((euler_temp.z() != e.z()))
      {
        euler_temp.z() = e.z();
      }
      send_data = false;
      robot_state = 1;
    }
    else if (payload == 0 && robot_state == 0)
    {
      robot_state = 4;
    }
  }
  euler.x() = abs(euler_temp.x() - e.x());
  euler.y() = euler_temp.y() - e.y();
  euler.z() = euler_temp.z() - e.z();

  if (state == 0)
  {
    motor1_controller.spin(0);
    motor2_controller.spin(0);
    motor3_controller.spin(0);
    lastPosition = 0;
    robot_state = 0;
    send_data = false;
  }
  else if (state == 1)
  {
    if (robot_state == 0)
    {
      if (position == 28)
      {
        motor1_controller.spin(0);
        motor2_controller.spin(0);
        motor3_controller.spin(0);
        send_data = true;
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
    else if (robot_state == 1)
    {
      float diff = calculateDegree(euler.x(), 270.0);
      // Serial.printf("euler x: %f abs_diff: %f diff: %f\n", euler.x(), diff, euler_temp.x());
      if (diff <= -5)
      {
        motor1_controller.spin(800);
        motor2_controller.spin(-800);
        motor3_controller.spin(-800);
      }
      else if (diff >= 5)
      {
        motor1_controller.spin(-800);
        motor2_controller.spin(800);
        motor3_controller.spin(800);
      }
      else
      {
        motor1_controller.spin(0);
        motor2_controller.spin(0);
        motor3_controller.spin(0);
        robot_state = 2;
      }
    }
    else if (robot_state == 2)
    {
      for (int i = 0; i < payload; i++)
      {
        if (tower_left == 0 && tower_right == 0)
        {
          break;
        }
        else if (tower_right >= tower_left && tower_right != 0)
        {
          servo_right.write(180);
          tower_right--;
        }
        else if (tower_left >= tower_right && tower_left != 0)
        {
          servo_left.write(90);
          tower_left--;
        }
        delay(500);
        servo_right.write(90);
        servo_left.write(180);
        delay(500);
      }
      robot_state = 3;
    }
    else if (robot_state == 3)
    {
      float diff = calculateDegree(euler.x(), 0.0);
      Serial.printf("euler x: %f abs_diff: %f diff: %f\n", euler.x(), diff, euler_temp.x());
      if (diff <= -5)
      {
        motor1_controller.spin(800);
        motor2_controller.spin(-800);
        motor3_controller.spin(-800);
      }
      else if (diff >= 5)
      {
        motor1_controller.spin(-800);
        motor2_controller.spin(800);
        motor3_controller.spin(800);
      }
      else
      {
        motor1_controller.spin(0);
        motor2_controller.spin(0);
        motor3_controller.spin(0);
        robot_state = 4;
      }
    }
    else if (robot_state == 4)
    {
      motor1_controller.spin(initial_speed);
      motor2_controller.spin(initial_speed);
      motor3_controller.spin(0);
      delay(100);
      payload = 0;
      robot_state = 0;
    }
  }
  else
  {
    state = 0;
  }
}

void onInputTask(void *pvParameters)
{
  while (true)
  {
  }
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

float calculateDegree(float degree_in, float degree_out)
{
  float diff = degree_in - degree_out;
  if (degree_out < 180)
  {
    if (diff > 180)
    {
      diff = -(360 - diff);
    }
  }
  else
  {
    if (diff < -180)
    {
      diff = 360 + diff;
    }
  }
  return diff;
}