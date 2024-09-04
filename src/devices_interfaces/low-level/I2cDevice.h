#ifndef __I2CDEVICE_H__
#define __I2CDEVICE_H__

#include <cstdint>
#include <cstddef>

class I2cDevice {
public:
    I2cDevice();
    ~I2cDevice();

    int8_t init();
    int8_t deinit();

    int8_t readByte(uint8_t addr, char * value, bool stop = true);
    int8_t readBytes(uint8_t addr, char * buf, size_t len, bool stop = true);
    int8_t writeByte(uint8_t addr, char value, bool stop = true);
    int8_t writeBytes(uint8_t addr, char * buf, size_t len, bool stop = true);
    int8_t writeAndReadBytes(uint8_t addr, char *write_buffer, size_t write_size, char *read_buffer, size_t read_size);
};

#endif // __I2CDEVICE_H__