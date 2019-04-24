/**
 * TMP2 library
 * author: Paul Leloup
 * update: 11-01-2018
 */

#include "system.h"
#include "i2c.h"

#pragma message("Compiling functions for TMP2")

short GetTemperature(void)
{
	unsigned char data[2] = {0};
	short temp;

	I2C1_Write((0x4B<<1), 1, data); //0x96
	I2C1_Read((0x4B<<1), 2, data);

	temp = (data[0] << 8) | (data[1] & 0xF8); //F8
	temp = temp >> 3;
	return temp;
}
