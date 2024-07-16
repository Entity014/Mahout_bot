#ifndef DRIVE_H
#define DRIVE_H

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2
        MOTOR3
         BACK
*/

#define USE_BTS7960_DRIVER
#define PWM_BITS 10         // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 20000 // PWM Frequency
#define PWM_MAX ((1 << PWM_BITS) - 1)
#define PWM_MIN ((1 << PWM_BITS) - 1) * -1

#define K_P 1.0
#define K_I 0.8
#define K_D 0

#define BTN 0

#define QTR1 36
#define QTR2 39
#define QTR3 34
#define QTR4 35
#define QTR5 32
#define QTR6 33
#define QTR7 25
#define QTR8 26

#define SERVO_LEFT 19
#define SERVO_RIGHT 18

#define MOTOR1_INV true
#define MOTOR2_INV true
#define MOTOR3_INV false

#define MOTOR1_PWM -1
#define MOTOR1_IN_A 17
#define MOTOR1_IN_B 5

#define MOTOR2_PWM -1
#define MOTOR2_IN_A 15
#define MOTOR2_IN_B 2

#define MOTOR3_PWM -1
#define MOTOR3_IN_A 4
#define MOTOR3_IN_B 16

#endif