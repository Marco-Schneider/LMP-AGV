#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <QTRSensors.h>
#include <BluetoothSerial.h>
#include <PID.h>
#include <ESP32Servo.h>

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
#define maxSpeed 200

/* Motors objects */
Motor leftMotor = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor rightMotor = Motor(BIN1, BIN2, PWMB, 1, STBY);

Servo servoMotor;

#define servoMotorPin 13

BluetoothSerial SerialBT;

QTRSensors lineSensor;

PIDController pid;

#define numberOfSensors 8
uint8_t lineSensorPins[numberOfSensors] = {34, 35, 32, 33, 25, 26, 27, 14};
uint16_t lineSensorValues[numberOfSensors];

void configureLineSensor();
void calibrateLineSensor();

// Discrete Sensors
#define leftSensor 39
#define rightSensor 36
#define loadSensor 15

bool leftSensorDetected = false;
bool rightSensorDetected = false;

int countLeft = 0;
int countRight = 0;

bool loaded = false;
bool unloading = false;

float map_line_position(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void checkLeftSensor() {
  leftSensorDetected = digitalRead(leftSensor) == LOW ?
    true : false;
}

void checkRightSensor() {
  rightSensorDetected = digitalRead(rightSensor) == LOW ?
    true : false;
}

void checkState() {
  loaded = loaded == true ? false : true;
}

int countMarkings = 0;

void setup() {
  SerialBT.begin("LMP-AGV-V1");

  configureLineSensor();
  calibrateLineSensor();
  pid.updateConstants(5.0, 0.0, 35.0);

  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);
  pinMode(loadSensor, INPUT);

  attachInterrupt(leftSensor, checkLeftSensor, RISING);
  attachInterrupt(rightSensor, checkRightSensor, RISING);
  attachInterrupt(digitalPinToInterrupt(loadSensor), checkState, FALLING);

  servoMotor.attach(servoMotorPin);
}

void loadingRoutine() {
  while(!loaded) {
    leftMotor.drive(0);
    rightMotor.drive(0);
    servoMotor.write(180);
    countMarkings = countMarkings == 0 ? countMarkings + 1 : countMarkings;
    delay(500);
  }
}

void loop() {
  float linePosition = map_line_position(lineSensor.readLineWhite(lineSensorValues), 0.0, 7000.0, -1.0, 1.0);
  float correction = pid.calculateCorrection(linePosition); 
  leftMotor.drive(constrain((1.0 - correction) * maxSpeed, (-1.0/5.0) * maxSpeed, maxSpeed));
  rightMotor.drive(constrain((1.0 + correction) * maxSpeed, (-1.0/5.0) * maxSpeed, maxSpeed)); 
  if(rightSensorDetected && !loaded) {
    loadingRoutine();
    delay(1500);
    servoMotor.write(90);
  }
  // SerialBT.println("left: ");
  // SerialBT.println(digitalRead(leftSensor));
  // SerialBT.println("right: ");
  // SerialBT.println(digitalRead(rightSensor));
  // SerialBT.println("load: ");
  // SerialBT.println(loaded);
  // SerialBT.println("left: ");
  // SerialBT.println(leftSensorDetected);
  // SerialBT.println("right: ");
  // SerialBT.println(rightSensorDetected);
  // SerialBT.println("countMarkings: ");
  // SerialBT.println(countMarkings);
  // delay(500);
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