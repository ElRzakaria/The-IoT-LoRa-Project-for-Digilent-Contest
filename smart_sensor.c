/**
 * Smart Sensor library
 * author: Paul Leloup
 * update: 25-01-2019
 */

#include "system.h"
#include "smart_sensor.h"

void cleanBufferMessage(void)
{
	int length = 0;
	while(bufferMessage[length] != '\0'){
		bufferMessage[length]='X';
		length++;
	}
}

void printBufferMessage(void)
{
	int length = 0;
	while(bufferMessage[length] != '\0'){
		length++;
	}
	printf("\n");
	for(int m = 0 ; m < length ; m++){
		printf("%c",bufferMessage[m]);
	}
}

void printDataSensorMessage(void)
{
	int i=0,virgule =0;
	while(bufferMessage[i] != '*')
	{
		if(bufferMessage[i]==',')
		{
			virgule++;
		}
		//humidité
		if(bufferMessage[i] == '#')
		{
			i=i+2;
			printf("\n");
			printf("Humidité : ");
			for(int m = 0 ; m < 6 ; m++){
				printf("%c",bufferMessage[i++]);
			}
			printf(" %%");
			i--;
		}
		//luminosité
		if(virgule==1)
		{
			i=i+2;
			printf("\n");
			printf("Luminosité : ");
			for(int m = 0 ; m < 6 ; m++){
				printf("%c",bufferMessage[i++]);
			}
			printf(" %%");
			i--;
		}
		//temperature
		if(virgule==2)
		{
			i=i+2;
			printf("\n");
			printf("Température : ");
			for(int m = 0 ; m < 6 ; m++){
				printf("%c",bufferMessage[i++]);
			}
			printf(" °C");
			i--;
		}
		//heure
		if(virgule==3)
		{
			i=i+2;
			printf("\n");
			printf("Heure : ");
			for(int m = 0 ; m < 6 ; m++){
				printf("%c",bufferMessage[i++]);
				if( (m==1) || (m==3) ){
					printf(":");
				}
			}
			i--;
		}
		//latitude
		if(virgule==4)
		{
			printf("\n");
			printf("Latitude : ");
			if(bufferMessage[i+1]!=',')
			{
				i=i+4;
				for(int m = 0 ; m < 10 ; m++){
					printf("%c",bufferMessage[i++]);
				}
				i--;
			}else{
				printf("Pas de réseau GPS");
			}
		}
		//longitude
		if(virgule==5)
		{
			printf("\n");
			printf("Longitude : ");
			if(bufferMessage[i+1]!=',')
			{
				i=i+4;
				for(int m = 0 ; m < 11 ; m++){
					printf("%c",bufferMessage[i++]);
				}
				i--;
			}else{
				printf("Pas de réseau GPS");
			}
		}
		//longitude
		if(virgule==6)
		{
			i=i+2;
			printf("\n");
			printf("Heure : ");
			for(int m = 0 ; m < 6 ; m++){
				printf("%c",bufferMessage[i++]);
				if( (m==1) || (m==3) ){
					printf("/");
				}
			}
			i--;
		}
		i++;
	}
}
