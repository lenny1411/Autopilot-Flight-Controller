//
// Created by lenny on 05/01/24.
//

#include "Imu.h"
#include <esp_log.h>
#include <utils/utils.h>

Imu::~Imu() {

}

int8_t Imu::init() {
    i2c->init();
    if(accel.init() != 0)
        return -1;
    if(gyro.init() != 0)
        return -1;
    if(mag.init() != 0)
        return -1;

    delay_milis(3000);

    imuCalibration(IMU_CALIBRATION_DURATION);
    magCalibration(MAG_CALIBRATION_DURATION);

    return 0;
}

int8_t Imu::deinit() {
    return 0;
}

int8_t Imu::updateAndGetData(struct attitudeData &values) {
    gyro.updateAndGetData(values);
    accel.updateAndGetData(values);

    values.gyroRateRoll -= gyroRateOffsetRoll; // deg/s
    values.gyroRatePitch -= gyroRateOffsetPitch; // deg/s
    values.gyroRateYaw -= gyroRateOffsetYaw; // deg/s

    values.accRateRoll -= accRateOffsetRoll;
    values.accRatePitch -= accRateOffsetPitch;
    values.accRateYaw -= accRateOffsetYaw;

    // values.gyroRateRoll = magMaxAxisX; // deg/s
    // values.gyroRatePitch = magMaxAxisY; // deg/s
    // values.gyroRateYaw = magMaxAxisZ; // deg/s

    // values.accRateRoll = magMinAxisX;
    // values.accRatePitch = magMinAxisY;
    // values.accRateYaw = magMinAxisZ;

    return 0;
}

int8_t Imu::getMagData(struct attitudeData &values)
{
    if(mag.updateAndGetData(values) == 0) {
        values.magY += compass_offset_y;                              //Add the y-offset to the raw value.
        values.magY *= compass_scale_y;                               //Scale the y-value so it matches the other axis.
        values.magZ += compass_offset_z;                              //Add the z-offset to the raw value.
        values.magZ *= compass_scale_z;                               //Scale the z-value so it matches the other axis.
        values.magX += compass_offset_x;                              //Add the x-offset to the raw value.
        return 0;
    }
    return -1;
}

Imu::Imu(I2cDevice *i2c) {
    this->i2c = i2c;
    accel = Bmi088Accel(*i2c, 0x19);
    gyro = Bmi088Gyro(*i2c, 0x69);
    mag = IST8310(i2c);

    gyroRateOffsetRoll  = 0;
    gyroRateOffsetPitch = 0;
    gyroRateOffsetYaw   = 0;
    accRateOffsetRoll   = 0;
    accRateOffsetPitch  = 0;
    accRateOffsetYaw    = 0;
}

void Imu::imuCalibration(uint16_t calibNum) {
    struct attitudeData values;
    for (size_t i = 0; i < calibNum; i++)
    {
        gyro.updateAndGetData(values);

        gyroRateOffsetRoll += values.gyroRateRoll;
        gyroRateOffsetPitch += values.gyroRatePitch;
        gyroRateOffsetYaw += values.gyroRateYaw;

        delay_milis(4);
    }
    gyroRateOffsetRoll /= calibNum;
    gyroRateOffsetPitch /= calibNum;
    gyroRateOffsetYaw /= calibNum;
}

void Imu::magCalibration(uint16_t time) {
    struct attitudeData values;
#if ENABLE_MAG_CALIBRATION == 1
    ESP_LOGE("MAG", "Calib start");

	while(mag.updateAndGetData(values) != 0);          					  //Read the raw compass values.

	magMinAxisX = values.magX;
	magMaxAxisX = values.magX;

	magMinAxisY = values.magY;
	magMaxAxisY = values.magY;

	magMinAxisZ = values.magZ;
	magMaxAxisZ = values.magZ;

    delay_milis(100);

	for(int i = 0;i < time;i++) {                                                 //Stay in this loop until the pilot lowers the pitch stick of the transmitter.                                                 //Send telemetry data to the ground station.
	    while(mag.updateAndGetData(values) != 0);          					  //Read the raw compass values.

        ESP_LOGE("MAG", "X=%i Y=%i Z=%i", values.magX, values.magY, values.magZ);

	    //In the following lines the maximum and minimum compass values are detected and stored.
	    if (values.magX < magMinAxisX)
	    	magMinAxisX = values.magX;
	    if (values.magX > magMaxAxisX)
	    	magMaxAxisX = values.magX;

	    if (values.magY < magMinAxisY)
	    	magMinAxisY = values.magY;
	    if (values.magY > magMaxAxisY)
	    	magMaxAxisY = values.magY;

	    if (values.magZ < magMinAxisZ)
	    	magMinAxisZ = values.magZ;
	    if (values.magZ > magMaxAxisZ)
	    	magMaxAxisZ = values.magZ;

        delay_milis(50);
	}
#endif


    compass_scale_y = ((float)magMaxAxisX - magMinAxisX) / (magMaxAxisY - magMinAxisY);
    compass_scale_z = ((float)magMaxAxisX - magMinAxisX) / (magMaxAxisZ - magMinAxisZ);

    compass_offset_x = (magMaxAxisX - magMinAxisX) / 2 - magMaxAxisX;
    compass_offset_y = (((float)magMaxAxisY - magMinAxisY) / 2 - magMaxAxisY) * compass_scale_y;
    compass_offset_z = (((float)magMaxAxisZ - magMinAxisZ) / 2 - magMaxAxisZ) * compass_scale_z;

    ESP_LOGE("MAG", "maxX=%i minX=%i", magMaxAxisX, magMinAxisX);
    ESP_LOGE("MAG", "maxY=%i minY=%i", magMaxAxisY, magMinAxisY);
    ESP_LOGE("MAG", "maxZ=%i minZ=%i", magMaxAxisZ, magMinAxisZ);
}