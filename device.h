#ifndef DEVICE_H
#define DEVICE_H

#include "dcmotor.h"
#include "servoremote.h"
#include "TinyMPU6050.h"

// Pins
#define RED_LED_PIN A1
#define YELLOW_LED_PIN A2
#define GREEN_LED_PIN A3

extern MPU6050 mpu;
extern DCMotor leftMotor, rightMotor;
extern ServoRemote xJoystick, yJoystick;

#endif