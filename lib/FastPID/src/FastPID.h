#ifndef FastPID_H
#define FastPID_H

#include <stdint.h>

/*
  A fixed point PID controller with a 32-bit internal calculation pipeline.
*/
class FastPID {

public:
  FastPID() 
  {
    clear();
  }

  FastPID(float kp, float ki, float kd, float hz)
  {
    configure(kp, ki, kd, hz);
  }

  ~FastPID();

  bool setCoefficients(float kp, float ki, float kd, float hz);
  bool setOutputConfig(int bits, bool sign);
  bool setOutputRange(float min, float max);
  void clear();
  bool configure(float kp, float ki, float kd, float hz);
  float step(float sp, float fb);

private:
  float kp;
  float ki;
  float kd;

  float myInput;
  float myOutput;
  float mySetpoint;

  float outMin, outMax;
  float error, errorSum, deltaError, lastError;
};
#endif
