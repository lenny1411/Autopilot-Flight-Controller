#include "FastPID.h"
#include <Arduino.h>
#include "../../../src/utils/utils.h"

FastPID::~FastPID() {
}

void FastPID::clear() {
  lastError = 0;
  errorSum = 0;
} 

bool FastPID::setCoefficients(float kp_, float ki_, float kd_, float hz) {
  if (kp_ < 0 || ki_ < 0 || kd_ < 0) return false;
  if (ki_ == 0) errorSum = 0;
  float SampleTimeSec = 1.0f / hz;
  kp = kp_;
  ki = ki_ * SampleTimeSec;
  kd = kd_ / SampleTimeSec;
  return true;
}

bool FastPID::setOutputRange(float min, float max)
{
  if (min >= max) return false;
  outMin = min;
  outMax = max;
  return true;
}

bool FastPID::configure(float kp, float ki, float kd, float hz) {
  clear();
  setCoefficients(kp, ki, kd, hz);
  return true; 
}

float FastPID::step(float sp, float fb) {
  mySetpoint = sp;
  myInput = fb;

  error = mySetpoint - myInput;

  // Prévention du windup sur l'intégrale
  if (errorSum * ki <= outMax && errorSum * ki >= outMin) {
    errorSum += error;
  }

  deltaError = error - lastError;
  lastError = error;

  // Contrainte de la somme des erreurs
  errorSum = constrain_(errorSum, outMin / ki, outMax / ki);

  // Calcul de la sortie PID
  myOutput = constrain_(error * kp + errorSum * ki + deltaError * kd, outMin, outMax);

  return myOutput;
}
