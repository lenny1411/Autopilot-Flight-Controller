/*
 * IST8310.cpp
 *
 *  Created on: 9 nov. 2023
 *      Author: lenny
 */

#include "IST8310.h"
#include "string.h"

IST8310::IST8310(I2cDevice* w) {
    this->w = w;
	isDataRequested = false;
}

void IST8310::setI2cInterface(I2cDevice* w) {
	this->w = w;
}

bool IST8310::isConnected(){
	uint8_t data = 0;
	read_regs((unsigned char)Register::WAI, (unsigned char*)&data, 1);
	if(data == 0x10)
		return true;
	return false;
}

int8_t IST8310::init() {
	if(!isConnected())
		return -1;

	uint8_t data = 0x01;

	for (int reset_count = 0; reset_count < 5; reset_count++) {
		data = 0x01;
		write_regs((unsigned char)Register::CNTL2, (unsigned char*)&data, 1);

	    delay_milis(10);

	    data = 0xFF;
	    read_regs((unsigned char)Register::CNTL2, (unsigned char*)&data, 1);

	    if ((data & 0x01) == 0) {
	        break;
	    }
	 }

	data = 0b11000000;
	write_regs((unsigned char)Register::PDCNTL, (unsigned char*)&data, 1);

	data = 0x24;
	write_regs((unsigned char)Register::AVGCNTL, (unsigned char*)&data, 1);

    delay_milis(10);

	return 0;
}

int8_t IST8310::deinit()
{
    return 0;
}

bool IST8310::isDataRdy() {
	uint8_t data = 0;
	read_regs((unsigned char)Register::STAT1, (unsigned char*)&data, 1);
	if((data & 0x01) == 1)
		return true;
	return false;
}

int8_t IST8310::read(int16_t* axisX, int16_t* axisY, int16_t* axisZ) {
	unsigned char buffer[6];

	read_regs((unsigned char)Register::DATAXL, (unsigned char*)buffer, 6);
	*axisY = (buffer[1] << 8) | buffer[0];
	*axisX = (buffer[3] << 8) | buffer[2];
	*axisZ = (buffer[5] << 8) | buffer[4];
	return 0;
}

int8_t IST8310::write_regs(uint8_t reg_addr,uint8_t *data, uint16_t len) {
	char buf[100];
	buf[0] = (char)reg_addr;
	memcpy(buf + 1, data, len);
	if(w->writeBytes(DEVICE_ADDR, buf, len + 1) != 0) {
	    return -1;
	}
	return 0;
}

int8_t IST8310::updateAndGetData(struct attitudeData &values)
{
	dataRequest();
	if(!isDataRdy())
		return -1;
	read(&values.magX, &values.magY, &values.magZ);
	isDataRequested = false;
	return 0;
}

void IST8310::dataRequest()
{
    uint8_t data = 0x01;
	write_regs((unsigned char)Register::CNTL1, (unsigned char*)&data, 1);
}

int8_t IST8310::read_regs(uint8_t reg_addr,uint8_t *data, uint16_t len) {
    w->writeAndReadBytes(DEVICE_ADDR, (char *)&reg_addr, 1,(char *)data, len);

	return 0;
}
