#include "device.h"
#include <Wire.h>

enum {
  // MPU6050 (accelerometer) works
  ASSISTED,   //manual control
  AUTONOMOUS, //signal lost, return to base

  // MPU6050 doesn't work
  MANUAL,        //manual control
  EMERGENCY_STOP //signal lost, stop the boat
} mode;

#define X_JOYSTICK_MID 108
#define X_JOYSTICK_MIN 0
#define X_JOYSTICK_MAX 230

#define Y_JOYSTICK_MID 108
#define Y_JOYSTICK_MIN 10
#define Y_JOYSTICK_MAX 220

uint8_t scanI2CForMPU6050() {
  Wire.begin();
  for(uint8_t address = 0x68; address <= 0x69; ++address) {
    Wire.beginTransmission(address);
    if(Wire.endTransmission() == 0)
      return address;
  }
  return 0;
}

void setup() {

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, INPUT); //green led is to bright, use internal 4.7kOhm pullup
  pinMode(YELLOW_LED_PIN, OUTPUT);

  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(YELLOW_LED_PIN, HIGH);

  delay(1000);

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);

  delay(1000);
  
  Serial.begin(500000);

  Serial.print(F("Initializing remote control.\n"));
  xJoystick.begin([](){xJoystick.isr();});
  yJoystick.begin([](){yJoystick.isr();});

  Serial.print(F("Scanning I2C bus... "));
  uint8_t address = scanI2CForMPU6050();
  if(address) {
    Serial.print(F("Found MPU6050 at address 0x"));
    Serial.print(address, HEX);
    Serial.write('\n');
    mpu = MPU6050(Wire, address);

    Serial.print(F("Initializing MPU6050... "));
    mpu.Initialize();
    Serial.print(F("Done.\nCallibrating MPU6050..."));
    digitalWrite(YELLOW_LED_PIN, HIGH);
    mpu.Calibrate();
    digitalWrite(YELLOW_LED_PIN, LOW);
    Serial.print(F("Done.\n"));
    mode = ASSISTED;
  } else {
    Serial.print(F("ERROR: MPU6050 not found, switching to manual mode.\n"));
    digitalWrite(RED_LED_PIN, HIGH);
    mode = MANUAL;
  }

  Serial.print(F("Waiting for remote control signal..."));
  while(!(xJoystick.hasSignal() && yJoystick.hasSignal())) {
    xJoystick.update(0);
    yJoystick.update(0);
    digitalWrite(GREEN_LED_PIN, HIGH);
    delay(250);
    digitalWrite(GREEN_LED_PIN, LOW);
    delay(250);
  }
  Serial.print(F("Done.\n"));
  digitalWrite(GREEN_LED_PIN, HIGH);

  Serial.print(F("Initializing motors.\n"));
  leftMotor.begin();
  rightMotor.begin();

  /*pinMode(HALL_SENSOR, INPUT);
  digitalWrite(HALL_SENSOR, INPUT_PULLUP);
  PCICR |= (1<<PCIE1);
  PCMSK1 |= (1<<PCINT8);
  sei();*/
}

int joystickPos(ServoRemote &remote, uint8_t minval, uint8_t midval, uint8_t maxval) {
  int value = remote.read();
  if(value > midval) value = map(value, midval, maxval, 0, 255);
  else value = map(value, midval, minval, 0, -255);
  return value;
}

unsigned long dt = 10000; //10 milliseconds

void loop() {

  unsigned long t = micros();

  int x = joystickPos(xJoystick, X_JOYSTICK_MIN, X_JOYSTICK_MID, X_JOYSTICK_MAX),
      y = joystickPos(yJoystick, Y_JOYSTICK_MIN, Y_JOYSTICK_MID, Y_JOYSTICK_MAX);
  bool hasSignal = xJoystick.hasSignal() && yJoystick.hasSignal();

  // State machine
  switch(mode) {
    
    case ASSISTED:
      if(hasSignal) {
        leftMotor.setSpeedAndDir(x+y);
        rightMotor.setSpeedAndDir(x-y);
      } else {
        Serial.print(F("Remote control signal lost, switching to autonomous mode.\n"));
        mode = AUTONOMOUS;
      }
    break;
    
    case AUTONOMOUS:
      if(hasSignal) {
        Serial.print(F("Detected remote control signal, switching to assisted mode.\n"));
        mode = ASSISTED;
      } else {
        // TODO: return to base
        leftMotor.setSpeedAndDir(0);
        rightMotor.setSpeedAndDir(0);
      }
    break;

    case MANUAL:
      if(hasSignal) {
        leftMotor.setSpeedAndDir(x+y);
        rightMotor.setSpeedAndDir(x-y);
      } else { 
        Serial.print(F("Remote control signal lost, stopping boat.\n"));
        mode = EMERGENCY_STOP;
      }
    break;

    case EMERGENCY_STOP:
      if(hasSignal) {
        Serial.print(F("Detected remote control signal, switching to manual mode.\n"));
        mode = MANUAL;
      } else { 
        leftMotor.setSpeedAndDir(0);
        rightMotor.setSpeedAndDir(0);
      }
  }

  leftMotor.update(dt);
  rightMotor.update(dt);
  xJoystick.update(dt);
  yJoystick.update(dt);

  /*mpu.Execute();
  Serial.print(mpu.GetAngX());
  Serial.write(' ');
  Serial.print(mpu.GetAngY());
  Serial.write(' ');
  Serial.print(mpu.GetAngZ());
  Serial.write('\n');*/

  dt = micros() - t;
}
