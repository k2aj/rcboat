#ifndef SERVOREMOTE_H
#define SERVOREMOTE_H 

#include <stdint.h>
#include <Arduino.h>

typedef uint8_t Pin;

/*  Initialization:
 *  
 *  ServoRemote remote(pin);
 *  void remoteIsr() {
 *    remote.isr();
 *  }
 *  
 *  void setup() {
 *    remote.begin(&remoteIsr);
 *  }
 */
class ServoRemote {
  public:

  ServoRemote(Pin pin);
  void begin(void (*isr)());
  void isr();
  void update(uint32_t dt);
  uint8_t read() const;
  bool hasSignal() const;

  private:
  Pin pin;
  volatile uint32_t risingEdge;
  volatile uint8_t value; //0 = error
  bool _hasSignal;
  volatile uint32_t timeSinceLastPulse;
};

#endif
