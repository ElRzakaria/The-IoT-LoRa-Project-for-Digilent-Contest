/**
 * ALS library
 * author: Paul Leloup
 * update: 08-01-2019
 */

#include "system.h"
#include "ALS.h"
#include "GPS.h"

#pragma message("Compiling functions for ALS")

unsigned char dataALS[2] = {0};

//Lire 2 octets par SPI
void readALS()
{
	SS2 = 0;
	SPI2_Transmit(2, dataALS, dataALS);
	SS2 = 1;
}

void initALS()
{
	unsigned char initALS[2] = {0};
	SS2 = 0;
	SPI2_Transmit(2, initALS, initALS);
	SS2 = 1;
}

void processMessageALS()
{
	unsigned int valuenombre , valuedecimal;
	unsigned char newvalue[6]= {0};
	int n = 0;
	long value;

	value = (long) (dataALS[1] >> 4) | (dataALS[0] << 4);
	value = (value*100);
	valuenombre = (value>>8);
	valuedecimal = _abs((100*value)/ 255)%100;

	newvalue[0] = 'L';
	newvalue[1] = (valuenombre/100) + '0';
	newvalue[2] = ((valuenombre%100)/10) + '0';
	newvalue[3] = (valuenombre%10) + '0';
	newvalue[4] = '.';
	newvalue[5] = ((valuedecimal%100)/10) + '0';
	newvalue[6] = (valuedecimal%10) + '0';


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
