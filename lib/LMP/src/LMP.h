#ifndef _LMP_h_
#define _LMP_h_

#include <stdint.h>

#define LED_BUILTIN 2

// TB6612FNG pinout
#define AIN1 18
#define BIN1 21
#define AIN2 5
#define BIN2 22
#define PWMA 2
#define PWMB 23
#define STBY 19

#define maxSpeed 255

#define servoMotorPin 13

#define numberOfSensors 8
uint8_t lineSensorPins[numberOfSensors] = {34, 35, 32, 33, 25, 26, 27, 14};

#define leftSensor 39
#define rightSensor 36
#define loadSensor 15

#endif