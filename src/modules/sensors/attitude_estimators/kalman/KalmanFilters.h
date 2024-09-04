#ifndef KALMAN_h
#define KALMAN_h
#include <math.h>
#include "Kalman.h"

//--------------------------------------------------------------------------------------------
// Variable declaration

class KalmanFilters {
public:
    KalmanFilters(float dt);
    void setAngles(float roll, float pitch, float yaw);
    void estimatedAngles(float roll, float pitch, float yaw, float rollRate, float pitchRate, float yawRate);
    void setParams(float Q, float R);
    float getRoll();
    float getPitch();
    float getYaw();
private:
    float roll, pitch, yaw;
    float dt, Q, R;
    float KalmanUncertaintyAngleRoll;
    float KalmanUncertaintyAnglePitch;
    float KalmanUncertaintyAngleYaw;

    Kalman kalmanRoll;
    Kalman kalmanPitch;
    Kalman kalmanYaw;

    void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
};

#endif