/**
 * OLED library
 * author: Paul Leloup
 * update: 08-01-2019
 */
/*
#include "system.h"
#include "OLED.h"
#include "SPI.h"

#pragma message("Compiling functions for OLED")

static unsigned char readRegOLED(unsigned char reg)
{
	unsigned char data[2] = {reg, 0};

	SS2 = 0;
	SPI2_Transmit(2, data, data);
	SS2 = 1;

	return data[1];
}

static void writeRegOLED(unsigned char value){
	unsigned char data[2] = {value};
	SS2 = 0;
	SPI2_Transmit(2, data, data);
	SS2 = 1;
}

void testOLED(unsigned char reg)
{
	printf("%02X", readRegOLED(reg));
}

void initOLED()
{
	resetOLED();
	VBATC = 1;
	writeRegOLED(0xAF);
	for (int i=0; i<100000; i++);
}

void resetOLED()
{
	RESET = 0;
	for (int i=0; i<1000; i++);
	RESET = 1;
}

*/
