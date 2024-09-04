#include "I2cDevice.h"
#include "config.h"
#include <Wire.h>
#include "driver/i2c.h"

I2cDevice::I2cDevice()
{
    
}

I2cDevice::~I2cDevice()
{
    deinit();
}

int8_t I2cDevice::init()
{
    i2c_config_t conf = {
        I2C_MODE_MASTER,
        SDA_PIN,
        SCL_PIN,
        GPIO_PULLUP_ENABLE,
        GPIO_PULLUP_ENABLE,
        400000
    };

    esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);

    if(ret != ESP_OK) return -1;

    ret = i2c_driver_install(I2C_NUM_0 , conf.mode, 0, 0, 0);

    if(ret != ESP_OK) return -1;

    return 0;
}

int8_t I2cDevice::deinit()
{
    return 0;
}

int8_t I2cDevice::readByte(uint8_t addr, char * value, bool stop)
{
    esp_err_t ret = i2c_master_read_from_device(I2C_NUM_0 , addr, (uint8_t *)value, 1, 10 / portTICK_PERIOD_MS);

    return ret == ESP_OK ? 0 : -1;
}

int8_t I2cDevice::readBytes(uint8_t addr, char * buf, size_t len, bool stop)
{
    esp_err_t ret = i2c_master_read_from_device(I2C_NUM_0, addr, (uint8_t *)buf, len, 10 / portTICK_PERIOD_MS);

    return ret == ESP_OK ? 0 : -1;
}

int8_t I2cDevice::writeByte(uint8_t addr, char value, bool stop)
{
    esp_err_t ret = i2c_master_write_to_device(I2C_NUM_0, addr, (uint8_t *)value, 1, 10 / portTICK_PERIOD_MS);

    return ret == ESP_OK ? 0 : -1;
}

int8_t I2cDevice::writeBytes(uint8_t addr, char * buf, size_t len, bool stop)
{
    esp_err_t ret = i2c_master_write_to_device(I2C_NUM_0, addr, (uint8_t *)buf, len, 10 / portTICK_PERIOD_MS);

    return ret == ESP_OK ? 0 : -1;
}

int8_t I2cDevice::writeAndReadBytes(uint8_t addr, char *write_buffer, size_t write_size, char *read_buffer, size_t read_size) {
    esp_err_t ret = i2c_master_write_read_device(I2C_NUM_0, addr, (uint8_t *)write_buffer, write_size, (uint8_t *)read_buffer, read_size, 10 / portTICK_PERIOD_MS);

    return ret == ESP_OK ? 0 : -1;
}

