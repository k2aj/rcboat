#include "dcmotor.h"
#include <Arduino.h>

DCMotor::DCMotor(Pin forward, Pin backward, Pin pwm) :
  forward(forward), backward(backward), pwm(pwm) {}
  
void DCMotor::begin() {
  digitalWrite(forward, LOW);
  digitalWrite(backward, LOW);
  digitalWrite(pwm, LOW);
  pinMode(forward, OUTPUT);
  pinMode(backward, OUTPUT);
  pinMode(pwm, OUTPUT);
  dir = nextDir = STOPPED;
}

void DCMotor::update(uint32_t dt) {
  if(safetyDelay > 0) {
    if(safetyDelay <= dt) {
      switch(dir) {
        case FORWARD: digitalWrite(forward, HIGH); break;
        case BACKWARD: digitalWrite(backward, HIGH); break;
        default: break;
      }
      safetyDelay = 0;
    } else safetyDelay -= dt;
  }
  if(nextDir != dir) {
    switch(dir) {
      case FORWARD: digitalWrite(forward, LOW); break;
      case BACKWARD: digitalWrite(backward, LOW); break;
      default: break;
    }
    dir = nextDir;
    safetyDelay = 100;
  }
}

void DCMotor::setSpeedAndDir(int value) {
  analogWrite(pwm, min(abs(value), 255));
  if(value < -10) nextDir = BACKWARD;
  else if(value > 10) nextDir = FORWARD;
  else nextDir = STOPPED;
}
