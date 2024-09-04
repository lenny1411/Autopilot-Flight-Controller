#include "KalmanFilters.h"

KalmanFilters::KalmanFilters(float dt) {
    this->dt = dt;
    // this->KalmanUncertaintyAngleRoll=4*2;
    // this->KalmanUncertaintyAnglePitch=4*2;
    // this->KalmanUncertaintyAngleYaw=4*2;
}

void KalmanFilters::setAngles(float roll, float pitch, float yaw) {
    // this->roll = roll;
    // this->pitch = pitch;
    // this->yaw = yaw;
    // this->KalmanUncertaintyAngleRoll=4*2;
    // this->KalmanUncertaintyAnglePitch=4*2;
    // this->KalmanUncertaintyAngleYaw=4*2;
    kalmanRoll.setAngle(roll); // Set starting angle
    kalmanPitch.setAngle(pitch);
    kalmanYaw.setAngle(yaw);
}

void KalmanFilters::estimatedAngles(float roll, float pitch, float yaw, float rollRate, float pitchRate, float yawRate) {
    // kalman_1d(this->roll, KalmanUncertaintyAngleRoll, rollRate, roll);
    // kalman_1d(this->pitch, KalmanUncertaintyAnglePitch, pitchRate, pitch);
    // kalman_1d(this->yaw, KalmanUncertaintyAngleYaw, yawRate, yaw);
   this->roll  = kalmanRoll.getAngle( roll,  rollRate,  this->dt);
   this->pitch = kalmanPitch.getAngle(pitch, pitchRate, this->dt);
   this->yaw = kalmanYaw.getAngle(yaw, yawRate, this->dt);
}

void KalmanFilters::setParams(float Q, float R) {
    // this->Q = Q;
    // this->R = R;
    kalmanRoll.setQangle(Q);
    kalmanPitch.setQangle(Q);
    kalmanYaw.setQangle(Q);
    kalmanRoll.setRmeasure(R);
    kalmanPitch.setRmeasure(R);
    kalmanYaw.setRmeasure(R);
}

float KalmanFilters::getRoll() {
    return roll;
}

float KalmanFilters::getPitch() {
    return pitch;
}

float KalmanFilters::getYaw() {
    return yaw;
}

void KalmanFilters::kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    KalmanState=KalmanState+dt*KalmanInput;
    KalmanUncertainty=KalmanUncertainty + dt * dt * Q * Q;
    float KalmanGain=KalmanUncertainty * 1.0f/(1.0f*KalmanUncertainty + R * R);
    KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
    KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
}
