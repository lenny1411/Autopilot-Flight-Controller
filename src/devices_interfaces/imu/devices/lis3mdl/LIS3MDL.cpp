#include "LIS3MDL.h"
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define LIS3MDL_SA1_HIGH_ADDRESS  0b0011110
#define LIS3MDL_SA1_LOW_ADDRESS   0b0011100

#define TEST_REG_ERROR -1

#define LIS3MDL_WHO_ID  0x3D

// Constructors ////////////////////////////////////////////////////////////////

LIS3MDL::LIS3MDL(void)
{
  _device = device_auto;
}

LIS3MDL::LIS3MDL(I2cDevice &w)
{
  _device = device_auto;
  this->w = w;
}

// Public Methods //////////////////////////////////////////////////////////////

bool LIS3MDL::init(deviceType device, sa1State sa1)
{
  // perform auto-detection unless device type and SA1 state were both specified
  if (device == device_auto || sa1 == sa1_auto)
  {
    // check for LIS3MDL if device is unidentified or was specified to be this type
    if (device == device_auto || device == device_LIS3MDL)
    {
      // check SA1 high address unless SA1 was specified to be low
      if (sa1 != sa1_low && testReg(LIS3MDL_SA1_HIGH_ADDRESS, WHO_AM_I) == LIS3MDL_WHO_ID)
      {
        sa1 = sa1_high;
        if (device == device_auto) { device = device_LIS3MDL; }
      }
      // check SA1 low address unless SA1 was specified to be high
      else if (sa1 != sa1_high && testReg(LIS3MDL_SA1_LOW_ADDRESS, WHO_AM_I) == LIS3MDL_WHO_ID)
      {
        sa1 = sa1_low;
        if (device == device_auto) { device = device_LIS3MDL; }
      }
    }

    // make sure device and SA1 were successfully detected; otherwise, indicate failure
    if (device == device_auto || sa1 == sa1_auto)
    {
      return false;
    }
  }

  _device = device;

  switch (device)
  {
    case device_LIS3MDL:
      address = (sa1 == sa1_high) ? LIS3MDL_SA1_HIGH_ADDRESS : LIS3MDL_SA1_LOW_ADDRESS;
      break;
  }

  return true;
}

/*
Enables the LIS3MDL's magnetometer. Also:
- Selects ultra-high-performance mode for all axes
- Sets ODR (output data rate) to default power-on value of 10 Hz
- Sets magnetometer full scale (gain) to default power-on value of +/- 4 gauss
- Enables continuous conversion mode
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LIS3MDL::enableDefault(void)
{
  if (_device == device_LIS3MDL)
  {
    // 0x70 = 0b01110000
    // OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR)
    writeReg(CTRL_REG1, 0x70);

    // 0x00 = 0b00000000
    // FS = 00 (+/- 4 gauss full scale)
    writeReg(CTRL_REG2, 0x00);

    // 0x00 = 0b00000000
    // MD = 00 (continuous-conversion mode)
    writeReg(CTRL_REG3, 0x00);

    // 0x0C = 0b00001100
    // OMZ = 11 (ultra-high-performance mode for Z)
    writeReg(CTRL_REG4, 0x0C);

    // 0x40 = 0b01000000
    // BDU = 1 (block data update)
    writeReg(CTRL_REG5, 0x40);
  }
}

// Writes a mag register
void LIS3MDL::writeReg(uint8_t reg, uint8_t value)
{
  char buf[2];
	buf[0] = (char)reg;
  buf[1] = (char)value;
  last_status = w.writeBytes(address, buf, 2);
}

// Reads a mag register
uint8_t LIS3MDL::readReg(uint8_t reg)
{
  uint8_t value;

  last_status = w.writeByte(address, (char)reg);
	w.readByte(address, (char *)&value);

  return value;
}

// Reads the 3 mag channels and stores them in vector m
void LIS3MDL::read()
{
  char buf[6];
  w.writeByte(address, OUT_X_L | 0x80);
  w.readBytes(address, buf, 6);
  uint8_t xlm = buf[0];
  uint8_t xhm = buf[1];
  uint8_t ylm = buf[2];
  uint8_t yhm = buf[3];
  uint8_t zlm = buf[4];
  uint8_t zhm = buf[5];

  // combine high and low bytes
  m.x = (int16_t)(xhm << 8 | xlm);
  m.y = (int16_t)(yhm << 8 | ylm);
  m.z = (int16_t)(zhm << 8 | zlm);
}

void LIS3MDL::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

int16_t LIS3MDL::testReg(uint8_t address, regAddr reg)
{
  if (w.writeByte(address, (char)reg) != 0)
  {
    return TEST_REG_ERROR;
  }

  uint8_t value;
  if (w.readBytes(address, (char *)&value, 1) == 0)
  {
    return value;
  }
  else
  {
    return TEST_REG_ERROR;
  }
}