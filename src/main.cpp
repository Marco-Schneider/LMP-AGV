#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <QTRSensors.h>
#include <PID.h>
#include <LMP.h>
#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// #include <BluetoothSerial.h>

Motor leftMotor = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor rightMotor = Motor(BIN1, BIN2, PWMB, 1, STBY);

Servo servoMotor;

QTRSensors lineSensor;
PIDController pid;
uint16_t lineSensorValues[numberOfSensors];

float line_position = 0;
float correction = 0;

void configureLineSensor();
void calibrateLineSensor();

bool leftSensorDetected = false;
bool rightSensorDetected = false;

bool loaded = false;
bool stopMotors = false;

/* Here are the scope definitions of the tasks to be used by the robot */
void read_sensors(void* parameters);
void unloading_routine(void* parameters);
void loading_routine(void* parameters);
void control_loop(void* parameters);
void motors_actuation(void* parameters);

SemaphoreHandle_t xLinePositionSemaphore;

float map_line_position(float x, float inMin, float inMax, float outMin, float outMax);

void checkLeftSensor() {
  leftSensorDetected = digitalRead(leftSensor) == HIGH;
}

void checkRightSensor() {
  rightSensorDetected = digitalRead(rightSensor) == HIGH;
}

void checkState() {
  loaded = loaded == true ? false : true;
}

// BluetoothSerial SerialBT;

void setup() {

  Serial.begin(115200);
  // SerialBT.begin("LMP-AGV-V2");

  configureLineSensor();
  calibrateLineSensor();
  pid.updateConstants(1.2, 0.0, 65.0);

  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);
  pinMode(loadSensor, INPUT);

  servoMotor.attach(servoMotorPin);

  attachInterrupt(leftSensor, checkLeftSensor, RISING);
  attachInterrupt(digitalPinToInterrupt(rightSensor), checkRightSensor, RISING);
  attachInterrupt(digitalPinToInterrupt(loadSensor), checkState, FALLING);

  xLinePositionSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(xLinePositionSemaphore);

  /* 
    Creating the tasks to be used by the robot, based on previous research, we have the following:

    1 - reading_sensors - To be run on the core 0 - Priority 3
    2 - control_loop - To be run on the core 1 - Priority 5
    3 - motors_actuation - To be run on the core 1 - Priority 4

  */

  xTaskCreatePinnedToCore(
    &read_sensors,
    "reading_sensors",
    4096,
    NULL,
    3,
    NULL,
    PRO_CPU_NUM
  );

  xTaskCreatePinnedToCore(
    &control_loop,
    "control_loop",
    4096,
    NULL,
    5,
    NULL,
    APP_CPU_NUM
  );

  xTaskCreatePinnedToCore(
    &motors_actuation,
    "running_motors",
    4096,
    NULL,
    4,
    NULL,
    APP_CPU_NUM
  );
}

void loadingRoutine() {
  while(!loaded) {
    servoMotor.write(180);
    delay(5);
  }
}

void unloadingRoutine() {
  while(loaded) {
    servoMotor.write(0);
    delay(5);
  }
}

void loop() { }

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
}

/* Reads and updates the variables related to each one of the sensors */
void read_sensors(void* parameters) {
  for(;;) {
    // Serial.println("\nread_sensors");
    // SerialBT.println(loaded);
    line_position = map_line_position(
      lineSensor.readLineWhite(lineSensorValues), 0.0, 7000.0, -1.0, 1.0
    );
    rightSensorDetected = digitalRead(rightSensor) == 1; 
    leftSensorDetected = digitalRead(leftSensor) == 1;
    // Serial.println("SENSOR\n");
    vTaskDelay(1);
  }
  vTaskDelete(NULL);
}

void control_loop(void* parameters) {
  const TickType_t xFrequency = 2 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;) {
    if(xSemaphoreTake(xLinePositionSemaphore, pdMS_TO_TICKS(2)) == true) {
      // Serial.println("\ncontrol_loop");
      correction = pid.calculateCorrection(line_position);
      if(rightSensorDetected && !loaded) {
        stopMotors = true;
        loadingRoutine();
        delay(15);
        servoMotor.write(90);
        // leftMotor.drive(100);
        // rightMotor.drive(100);
        delay(2);
        rightSensorDetected = false;
        stopMotors = false;
      }
      if(rightSensorDetected && loaded) {
        stopMotors = true;
        unloadingRoutine();
        delay(15);
        servoMotor.write(90);
        // leftMotor.drive(100);
        // rightMotor.drive(100);
        delay(2);
        rightSensorDetected = false;
        stopMotors = false;
      }
      // Serial.println("CONTROLE\n");
      xSemaphoreGive(xLinePositionSemaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  vTaskDelete(NULL);
}

void motors_actuation(void* parameters) {
  for(;;) {
    if(xSemaphoreTake(xLinePositionSemaphore, pdMS_TO_TICKS(2)) == true) {
      // SerialBT.println(stopMotors); <--- WHEN THIS was active
      if(!stopMotors) {
        leftMotor.drive(constrain((1.0 - correction) * maxSpeed, (-1.0/5.0) * maxSpeed, maxSpeed));
        rightMotor.drive(constrain((1.0 + correction) * maxSpeed, (-1.0/5.0) * maxSpeed, maxSpeed)); 
      }
      else {
        leftMotor.drive(0);
        rightMotor.drive(0);
      }
      // Serial.println("\nmotors_actuation");
      // Serial.println("MOTOR\n");
      xSemaphoreGive(xLinePositionSemaphore);
    }
    vTaskDelay(1);
  }
  vTaskDelete(NULL);
}

float map_line_position(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}