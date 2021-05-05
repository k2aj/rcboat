#include "device.h"
#include <Wire.h>

// More pins
#define LEFT_FORWARD_PIN 5
#define LEFT_BACKWARD_PIN 6
#define LEFT_PWM_PIN 9
#define RIGHT_FORWARD_PIN 7
#define RIGHT_BACKWARD_PIN 8
#define RIGHT_PWM_PIN 10

#define Y_JOYSTICK_PIN 2
#define X_JOYSTICK_PIN 3

ServoRemote xJoystick(X_JOYSTICK_PIN), 
            yJoystick(Y_JOYSTICK_PIN);
DCMotor leftMotor(LEFT_FORWARD_PIN,LEFT_BACKWARD_PIN,LEFT_PWM_PIN), 
        rightMotor(RIGHT_FORWARD_PIN,RIGHT_BACKWARD_PIN,RIGHT_PWM_PIN);
MPU6050 mpu(Wire);