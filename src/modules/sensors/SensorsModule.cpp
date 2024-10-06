//

// Created by lenny on 14/01/24.
//

#include "SensorsModule.h"

SensorsModule::SensorsModule(I2cDevice *i2c) {
    imu = Imu(i2c);
}

int8_t SensorsModule::init() {
    if(imu.init() != 0)
        return -1;

    MessageManager::getInstance().subscribe(SENSOR_CONFIG_TOPIC, [this](const void* message) -> void {
        config = *(static_cast<const attitudeConfig *>(message));
        ahrs.setConfig(config.param1);
    });

    MessageManager::getInstance().subscribe(STATE_TOPIC, [this](const void* message) -> void {
        state = *(static_cast<const droneState *>(message));
    });

    delay_milis(3000);

    // imu.updateAndGetData(attitudeValues);
    // while(imu.getMagData(attitudeValues) != 0);

    // float roll, pitch;
    // computeOrientation(attitudeValues.accRateRoll, attitudeValues.accRatePitch, attitudeValues.accRateYaw, &roll, &pitch);
    // headingCorrection = computeHeading(attitudeValues.magX, attitudeValues.magY, attitudeValues.magZ, roll, pitch);

    // ahrs.begin(ATTITUDE_LOOP_FREQ, roll, pitch, headingCorrection);
    // ahrs.setConfig(PARAM_1, PARAM_2);

    // kalman.setParams(PARAM_1, PARAM_2);
    // kalman.setAngles(roll, pitch, headingCorrection);

    ahrs.setConfig(PARAM_1);
    prevTimestamp = get_ms_count();

    return 0;
}

void SensorsModule::run() {
    timestamp = get_ms_count();

    getDataFromSensors();

    computeData();

    MessageManager::getInstance().publish(SENSOR_TOPIC, &attitudeValues);

    attitudeValues.loopPeriod = get_ms_count() - timestamp;
    wait(attitudeValues.loopPeriod, ATTITUDE_LOOP_FREQ);
    count++;
}

void SensorsModule::getDataFromSensors() {
    imu.updateAndGetData(attitudeValues);
    if(state == DISARMED) imu.getMagData(attitudeValues);
}

void SensorsModule::computeData() {
    float roll, pitch;

    attitudeValues.accRateRoll  -= config.offsetRoll;
    attitudeValues.accRatePitch -= config.offsetPitch;
    attitudeValues.accRateYaw   -= config.offsetYaw;

    if(state != DISARMED) {
        ahrs.updateIMU(
                attitudeValues.gyroRateRoll,
                attitudeValues.gyroRatePitch,
                attitudeValues.gyroRateYaw,
                attitudeValues.accRateRoll,
                attitudeValues.accRatePitch,
                attitudeValues.accRateYaw
                // attitudeValues.magX, 
                // attitudeValues.magY, 
                // attitudeValues.magZ
        );
        // kalman.estimatedAngles(roll, pitch, headingCorrection, attitudeValues.gyroRateRoll, attitudeValues.gyroRatePitch, -attitudeValues.gyroRateYaw);

        attitudeValues.roll    = ahrs.getRoll();
        attitudeValues.pitch   = ahrs.getPitch();
        attitudeValues.heading = map_(ahrs.getYaw(), 0, 360, 360, 0);
    } else {
        computeOrientation(attitudeValues.accRateRoll, attitudeValues.accRatePitch, attitudeValues.accRateYaw, &roll, &pitch);
        headingCorrection = computeHeading(attitudeValues.magX, attitudeValues.magY, attitudeValues.magZ, roll, -pitch);

        //kalman.setAngles(roll, pitch, headingCorrection);
        ahrs.begin(ATTITUDE_LOOP_FREQ, roll, pitch, map_(headingCorrection, 0, 360, 360, 0));

        attitudeValues.roll    = roll;
        attitudeValues.pitch   = pitch;
        attitudeValues.heading = headingCorrection;
    }

    // attitudeValues.roll    = kalman.getRoll();
    // attitudeValues.pitch   = kalman.getPitch();
    // attitudeValues.heading = ahrs.getYaw();

    // attitudeValues.roll    = ahrs.getRoll();
    // attitudeValues.pitch   = ahrs.getPitch();
    // attitudeValues.heading = ahrs.getYaw();

    if(attitudeValues.heading > 360)
        attitudeValues.heading -= 360;
    if(attitudeValues.heading < 0)
        attitudeValues.heading += 360;

    attitudeValues.yaw = attitudeValues.heading;

    if(attitudeValues.yaw > 180)
        attitudeValues.yaw -= 360;
}

float SensorsModule::computeHeading(int16_t magAxisX, int16_t magAxisY, int16_t magAxisZ, float roll, float pitch) {
    float compass_x_horizontal, compass_y_horizontal, actual_compass_heading;
    //The compass values change when the roll and pitch angle of the quadcopter changes. That's the reason that the x and y values need to calculated for a virtual horizontal position.
    //The 0.0174533 value is phi/180 as the functions are in radians in stead of degrees.
    compass_y_horizontal = (float)magAxisX * sin(roll * 0.0174533) * sin(pitch * 0.0174533) + (float)magAxisY * cos(roll * 0.0174533) - (float)magAxisZ * sin(roll * 0.0174533) * cos(pitch * 0.0174533);
    compass_x_horizontal = (float)magAxisX * cos(pitch * 0.0174533) + (float)magAxisZ * sin(pitch * 0.0174533);

    //Now that the horizontal values are known the heading can be calculated. With the following lines of code the heading is calculated in degrees.
    //Please note that the atan2 uses radians in stead of degrees. That is why the 180/3.14 is used.
    if (compass_y_horizontal < 0)
        actual_compass_heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14)));
    else
        actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);

    actual_compass_heading += MAGNETIC_DECLINATION;     //Add the declination to the magnetic compass heading to get the geographic north.
    if (actual_compass_heading < 0)
        actual_compass_heading += 360;         //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
    else if (actual_compass_heading >= 360)
        actual_compass_heading -= 360;         //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

    actual_compass_heading = map_(actual_compass_heading, 0, 360, 360, 0);

    return actual_compass_heading;
}

void SensorsModule::computeOrientation(float ax, float ay, float az, float *roll, float *pitch) {
    // Compute roll (phi) and pitch (theta) using accelerometer data
    *roll = atan2(ay, az) * 180.0 / M_PI;  // Convert to degrees
    *pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;  // Convert to degrees
}