#include "PID.h"
#include <Arduino.h>
#include <stdio.h>

PIDController::PIDController() {
  Kp = 0;
  Ki = 0;
  Kd = 0;
  setpoint = 0;
  error = 0;
  sumError = 0;
  lastError = 0;
}

void PIDController::updateConstants(float Kp, float Ki, float Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PIDController::updateSetpoint(float setpoint) {
  this->setpoint = setpoint;
}

float PIDController::calculateCorrection(float linePosition) {

  error = setpoint - linePosition;
  sumError += error;

  int64_t currentTime = millis();
  int64_t dt = currentTime - lastTime;


  float proportional_term = Kp * error;
  float derivative_term = Kd * (error - lastError)/dt;
  float integral_term = Ki * (sumError * dt);

  lastError = error;
  lastTime = currentTime;

  pid = proportional_term + derivative_term + integral_term;

  return pid;
}