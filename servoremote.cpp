#include "servoremote.h"

ServoRemote::ServoRemote(Pin pin) : pin(pin) {}

void ServoRemote::begin(void (*isr)()) {
  pinMode(pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin), isr, CHANGE);
  _hasSignal = false;
  timeSinceLastPulse = 1000000000; //1000 seconds
}

void ServoRemote::isr() {
  if(digitalRead(pin)) { 
    // Rising edge
    risingEdge = micros();
  } else {
    // Falling edge
    uint32_t pulseDuration = micros() - risingEdge;

    // Magic numbers would normally be 996 and 2004, but we use half of that because we run at half the clock speed of Arduino Uno
    if(pulseDuration >= 996uL && pulseDuration <= 2004uL) {
      value = ((pulseDuration - 996) >> 2) + 1;
      timeSinceLastPulse = 0;
    }
  }
}

void ServoRemote::update(uint32_t dt) {
  // Ensure interrupts don't mess things up
  cli();
  if(timeSinceLastPulse > 500000) //0.5s
    _hasSignal = false;
  else {
    timeSinceLastPulse += dt;
    _hasSignal = true;
  }
  sei();
}

uint8_t ServoRemote::read() const {
  return value;
}

bool ServoRemote::hasSignal() const {
  return _hasSignal;
}
