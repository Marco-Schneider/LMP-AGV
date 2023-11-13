#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <QTRSensors.h>
#include <BluetoothSerial.h>
#include <PID.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define LED_BUILTIN 2

// TB6612FNG pinout
#define AIN1 18
#define BIN1 21
#define AIN2 5
#define BIN2 22
#define PWMA 2
#define PWMB 23
#define STBY 19

// Speed
#define maxSpeed 255

/* Motors objects */
Motor leftMotor = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor rightMotor = Motor(BIN1, BIN2, PWMB, 1, STBY);

BluetoothSerial SerialBT;

QTRSensors lineSensor;

PIDController pid;

#define numberOfSensors 8
uint8_t lineSensorPins[numberOfSensors] = {34, 35, 32, 33, 25, 26, 27, 14};
uint16_t lineSensorValues[numberOfSensors];

void configureLineSensor();
void calibrateLineSensor();

float map_line_position(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void setup() {
  SerialBT.begin("LMP-AGV-V1");
  configureLineSensor();
  calibrateLineSensor();
  pid.updateConstants(20.0, 0.0, 0.0);
}

void loop() {
  float linePosition = map_line_position(lineSensor.readLineWhite(lineSensorValues), 0.0, 7000.0, -1.0, 1.0);
  float correction = pid.calculateCorrection(linePosition); 
  leftMotor.drive(constrain((1.0 - correction) * maxSpeed, (-1.0/5.0) * maxSpeed, maxSpeed));
  rightMotor.drive(constrain((1.0 + correction) * maxSpeed, (-1.0/5.0) * maxSpeed, maxSpeed)); 
  // SerialBT.print("linePosition: ");
  // SerialBT.println(linePosition);
  // SerialBT.print("leftMotor: ");
  // SerialBT.println(constrain((1.0 - correction) * maxSpeed, (-1.0/5.0) * maxSpeed, maxSpeed));
  // SerialBT.print("rightMotor: ");
  // SerialBT.println(constrain((1.0 + correction) * maxSpeed, (-1.0/5.0) * maxSpeed, maxSpeed));
}

void configureLineSensor() {
  lineSensor.setTypeAnalog();
  lineSensor.setSensorPins(lineSensorPins, numberOfSensors);
  lineSensor.setTimeout(1000);
  delay(1000);
}

void calibrateLineSensor() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for(uint16_t i=0; i<400; i++) {
    lineSensor.calibrate();
  }

  for(int i=0; i<5; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  SerialBT.println("Finished calibration!");
}

float constrain_new(float value, float min, float max) {
  if (value < min) {
    return min;
  } 
  else if (value > max) {
    return max;
  } 
  else {
    return value;
  }
}