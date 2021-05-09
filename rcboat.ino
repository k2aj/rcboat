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

#define X_JOYSTICK_MID 125
#define X_JOYSTICK_MIN 2
#define X_JOYSTICK_MAX 245

#define Y_JOYSTICK_MID 127
#define Y_JOYSTICK_MIN 10
#define Y_JOYSTICK_MAX 235

uint8_t scanI2CForMPU6050() {
  Wire.begin();
  Serial.print(F("Scanning I2C bus.\n"));
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

  Serial.begin(500000);

  Serial.print(F("Initializing remote control.\n"));
  xJoystick.begin([](){xJoystick.isr();});
  yJoystick.begin([](){yJoystick.isr();});

  auto t = millis();
  do {
    xJoystick.update(0);
    yJoystick.update(0);
  } while(millis() - t < 1000);

  uint8_t address;
  if(
    !(xJoystick.hasSignal() && yJoystick.hasSignal()) && 
    (address = scanI2CForMPU6050())
  ) {
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
    if(address) Serial.print(F("Remote control signal detected"));
    else Serial.print(F("ERROR: MPU6050 not found"));
    Serial.print(F(", switching to manual mode.\n"));
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

float sign(float x) {
  if(x < 0) return -1;
  else if(x == 0) return 0;
  else return 1;
}

unsigned long dt = 10000; //10 milliseconds
float targetAngle; //zakres <-180, 180>

void loop() {

  unsigned long t = micros();

  int x = joystickPos(xJoystick, X_JOYSTICK_MIN, X_JOYSTICK_MID, X_JOYSTICK_MAX),
      y = joystickPos(yJoystick, Y_JOYSTICK_MIN, Y_JOYSTICK_MID, Y_JOYSTICK_MAX);
  // It appears the remote control receiver will output y=5 or y=6 when signal is lost
  bool hasSignal = xJoystick.hasSignal() && yJoystick.hasSignal() && (yJoystick.read() > 8);

  // State machine
  switch(mode) {
    
    case ASSISTED:
      mpu.Execute();
      if(hasSignal) {
        leftMotor.setSpeedAndDir(y+x);
        rightMotor.setSpeedAndDir(y-x);
      } else {
        Serial.print(F("Remote control signal lost, switching to autonomous mode.\n"));
        mode = AUTONOMOUS;
        float curAngle = mpu.GetAngZ();
        targetAngle = curAngle - sign(curAngle)*180;
      }
    break;
    
    case AUTONOMOUS:
      mpu.Execute();
      if(hasSignal) {
        Serial.print(F("Detected remote control signal, switching to assisted mode.\n"));
        mode = ASSISTED;
      } else {
        float curAngle = mpu.GetAngZ();
        //how much extra rotation is needed to reach targetAngle
        // angleToReachTgt < 0 -> we need to rotate left
        // angleToReachTgt > 0 -> we need to rotate right
        int16_t angleToReachTgt = targetAngle - curAngle; 
        if(angleToReachTgt < -180) angleToReachTgt += 360;
        if(angleToReachTgt > 180) angleToReachTgt -= 360;

        leftMotor.setSpeedAndDir(100-angleToReachTgt);
        rightMotor.setSpeedAndDir(100+angleToReachTgt);
      }
    break;

    case MANUAL:
      if(hasSignal) {
        leftMotor.setSpeedAndDir(y+x);
        rightMotor.setSpeedAndDir(y-x);
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

  dt = micros() - t;
}
