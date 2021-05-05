#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <stdint.h>

typedef uint8_t Pin;

class DCMotor {
  public:
  DCMotor(Pin forward, Pin backward, Pin pwm);
  void begin();
  // 0 -> turned off, 255 -> max speed forward, -255 -> max speed backward
  void setSpeedAndDir(int value);
  void update(uint32_t dt);
  private:
  Pin forward, backward, pwm;
  enum {STOPPED, FORWARD, BACKWARD} dir, nextDir;
  uint8_t safetyDelay;
};

#endif
