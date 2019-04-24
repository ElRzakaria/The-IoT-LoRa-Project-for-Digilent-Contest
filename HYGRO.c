/**
 * HYGRO library
 * author: Paul Leloup
 * update: 11-01-2019
 */

#include <timer.h>
#include "system.h"
#include "HYGRO.h"
#include "GPS.h"
#include "i2c.h"

#pragma message("Compiling functions for HYGRO")


unsigned char dataHYGRO[4] = {0x00,0x00,0x00,0x00};

void initHYGRO(void)
{
	unsigned char data[1] = {0x00};
	sleep(15);
	I2C1_Write((0x80), 1, data);
}

void readHYGRO()
{
	for(int i=0;i<4;i++){
		dataHYGRO[i] = 0;
	}
	I2C1_Write((0x80), 1, dataHYGRO);
	sleep(250);
	I2C1_Read((0x80), 4, dataHYGRO);
}

void processMessageTemperatureHYGRO()
{
	unsigned int tempnombre , tempdecimal;
	unsigned char newvalue[7]= {0};
	int n = 0;
	long temp;

	temp = (dataHYGRO[0] << 8) | dataHYGRO[1];
	temp = (temp * 165);
	tempnombre =((temp>>16)-40);
	tempdecimal = _abs(((100*temp)/ 65536)-4000)%100;

	newvalue[0] = 'T';
	newvalue[1] = (tempnombre/100) + '0';
	newvalue[2] = ((tempnombre%100)/10) + '0';
	newvalue[3] = (tempnombre%10) + '0';
	newvalue[4] = '.';
	newvalue[5] = ((tempdecimal%100)/10) + '0';
	newvalue[6] = (tempdecimal%10) + '0';

	if(bufferMessage[0]=='#'){
		while(bufferMessage[n] != '*'){
			n++;
		}
		bufferMessage[n++]=',';
		for(int i=0;i<7;i++){
			bufferMessage[n++]=newvalue[i];
		}
		bufferMessage[n++]='*';
		bufferMessage[n++]='\0';
	}else{
		bufferMessage[n++]='#';
		for(int i=0;i<7;i++){
			bufferMessage[n++]=newvalue[i];
		}
		bufferMessage[n++]='*';
		bufferMessage[n++]='\0';
	}
}

void processMessageHumidityHYGRO()
{

	unsigned int humiditynombre , humiditydecimal;
	unsigned char newvalue[7]= {0};
	int n = 0;
	long humidity;

	humidity = (dataHYGRO[2] << 8) | dataHYGRO[3];
	humidity = (humidity * 100);
	humiditynombre = (humidity>>16);
	humiditydecimal = _abs((100*humidity)/ 65536)%100;

	newvalue[0] = 'H';
	newvalue[1] = (humiditynombre/100) + '0';
	newvalue[2] = ((humiditynombre%100)/10) + '0';
	newvalue[3] = (humiditynombre%10) + '0';
	newvalue[4] = '.';
	newvalue[5] = ((humiditydecimal%100)/10) + '0';
	newvalue[6] = (humiditydecimal%10) + '0';

	if(bufferMessage[0]=='#'){
		while(bufferMessage[n] != '*'){
			n++;
		}
		bufferMessage[n++]=',';
		for(int i=0;i<7;i++){
			bufferMessage[n++]=newvalue[i];
		}
		bufferMessage[n++]='*';
		bufferMessage[n++]='\0';
	}else{
		bufferMessage[n++]='#';
		for(int i=0;i<7;i++){
			bufferMessage[n++]=newvalue[i];
		}
		bufferMessage[n++]='*';
		bufferMessage[n++]='\0';
	}
}
