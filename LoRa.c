/**
 * LoRa library
 * author: Paul Leloup
 * update: 12-11-2018
 */

#include <timer.h>
#include "system.h"
#include "LoRa.h"
#include "LoRaFskReg.h"
#include "LoRaReg.h"
#include "SPI.h"

#pragma message("Compiling functions for LoRa")

static unsigned char readRegLoRa(unsigned char reg)
{
	unsigned char data[2] = {reg, 0};

	SS1 = 0;
	SPI1_Transmit(2, data, data);
	SS1 = 1;

	return data[1];
}

static void writeRegLora(unsigned char address, unsigned char value){
	unsigned char data[2] = {(address | 0x80), value};
	SS1 = 0;
	SPI1_Transmit(2, data, data);
	SS1 = 1;
}

void testLoRa(unsigned char reg)
{
	printf("%02X", readRegLoRa(reg));
}

void initLoRa()
{
	SS1 = 1;
	for (int i=0; i<10000; i++);

	//Reset LoRa Module
	resetLoRa();

	//LoRa Mode
	modeLoRa();

	//LoRa CRC OFF
	CRCLoRa(0); //ON

	//LoRa Frequency hopping ON
	frequencyHoppingLoRa(0); //ON

	//LoRa Spreading Factor 12 (4096 chips / symbol)
	SpreadingFactorLoRa(12);

	//LoRa PA BOOST OFF SETTINGS
	//PowerAmplifierSettingLoRa();

}

void resetLoRa()
{
	RESET1 = 1;
	for (int i=0; i<5000; i++);
	RESET1 = 0;
	for (int i=0; i<20000; i++);
}

void modeLoRa()
{
	writeRegLora(REG_LR_OPMODE, RFLR_OPMODE_SLEEP);
	writeRegLora(REG_LR_OPMODE, ((readRegLoRa(REG_LR_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_ON));
	writeRegLora(REG_LR_OPMODE, ((readRegLoRa(REG_LR_OPMODE) & RFLR_OPMODE_MASK) | RFLR_OPMODE_STANDBY));
}

void DIOMappingLoRa(int DIOMAP)
{
	if(DIOMAP == 0){
		writeRegLora(REG_LR_DIOMAPPING1,0x00); //RxDone
	}
	if(DIOMAP == 1){
		writeRegLora(REG_LR_DIOMAPPING1,0x40); //TxDone
	}
}

void CRCLoRa(int CRC)
{
	if(CRC == 1){
		writeRegLora(REG_LR_MODEMCONFIG1, ((readRegLoRa(REG_LR_MODEMCONFIG1) & RF_PACKETCONFIG1_CRC_MASK) | RF_PACKETCONFIG1_CRC_ON)); // ON
	}
	if(CRC == 0){
		writeRegLora(REG_LR_MODEMCONFIG1, ((readRegLoRa(REG_LR_MODEMCONFIG1) & RF_PACKETCONFIG1_CRC_MASK) | RF_PACKETCONFIG1_CRC_OFF)); //OFF
	}
}

void frequencyHoppingLoRa(int FH)
{
	if(FH == 1){
		writeRegLora(REG_LR_HOPPERIOD,0x11); // ON
	}
	if(FH == 0){
		writeRegLora(REG_LR_HOPPERIOD,0x00); //OFF
	}
}

void SpreadingFactorLoRa(int SF)
{
	//STANDBY MODE
	writeRegLora(REG_LR_OPMODE, ((readRegLoRa(REG_LR_OPMODE) & RFLR_OPMODE_MASK) | RFLR_OPMODE_STANDBY));

	if(SF < 6) {SF=6;}
	if(SF > 12) {SF=12;}

	if(SF==6){
		writeRegLora(REG_LR_DETECTOPTIMIZE,0xc5);
		writeRegLora(REG_LR_DETECTIONTHRESHOLD,0x0c);
	}else{
		writeRegLora(REG_LR_DETECTOPTIMIZE,0xc3);
		writeRegLora(REG_LR_DETECTIONTHRESHOLD,0x0a);
	}

	writeRegLora(REG_LR_MODEMCONFIG2,((readRegLoRa(REG_LR_MODEMCONFIG2) & 0x0F) | ((SF << 4) & 0xF0)));
}

void PowerAmplifierSettingLoRa()
{
	//STANDBY MODE
	writeRegLora(REG_LR_OPMODE, ((readRegLoRa(REG_LR_OPMODE) & RFLR_OPMODE_MASK) | RFLR_OPMODE_STANDBY));

	//20 dBm maximum on PA_BOOST pin
	writeRegLora(REG_PACONFIG, RFLR_PACONFIG_PASELECT_PABOOST | 2);

	//Default configuration (no boost)
	writeRegLora(REG_PADAC, RFLR_PADAC_20DBM_OFF);
}

int lengthLora(unsigned char *Package){
	int length = 0;
	while(Package[length] != '\0'){
		length++;
	}
	return length;
}

void SendPackageLoRa(unsigned char *Package, int packageLength)
{
	unsigned char datasend[1] = {(0x00 | 0x80)};
	unsigned char dummy[256];

	//STANDBY MODE
	writeRegLora(REG_LR_OPMODE, ((readRegLoRa(REG_LR_OPMODE) & RFLR_OPMODE_MASK) | RFLR_OPMODE_STANDBY));

	//DIOMAPPING TXDONE
	DIOMappingLoRa(1);

	//SET PAYLOAD LENGTH
	writeRegLora(REG_LR_PAYLOADLENGTH,packageLength);

	//SET FIFO POINTER
	writeRegLora(REG_LR_FIFOADDRPTR,RFLR_FIFOTXBASEADDR);

	//WRITE PAYLOAD IN THE FIFO
	SS1 = 0;
	SPI1_Transmit(1, datasend, dummy);
	SPI1_Transmit(packageLength, Package, dummy);
	SS1 = 1;

	//SWITCH TX MODE
	TXLORA1 = 0;
	RXLORA1 = 0;

	//TX MODE
	writeRegLora(REG_LR_OPMODE,((readRegLoRa(REG_LR_OPMODE) & RFLR_OPMODE_MASK) | RFLR_OPMODE_TRANSMITTER));

	//TXDONE DIO0 == 1
	while (!DIO0LORA1);

	//RESETING FLAG DIO0
	if(readRegLoRa(REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_TXDONE_MASK){
		writeRegLora(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_TXDONE);
	}

	//SWITCH STANDBY MODE
	TXLORA1 = 0;
	RXLORA1 = 0;
}

void ReceivePackageLoRa(char *Package)
{
	int packageLength = 0, last_pk_snr, last_pk_rssi, snr_dB, rssi_dB;
	//int tmax = 0, unread = 0;
	unsigned char datasend[256] = {(0x00 | 0x00)};
	unsigned char dataread[256] = {0x00};

	//RX MODE
	writeRegLora(REG_LR_OPMODE,((readRegLoRa(REG_LR_OPMODE) & RFLR_OPMODE_MASK) | RFLR_OPMODE_RECEIVER));

	//DIOMAPPING RXDONE
	DIOMappingLoRa(0);

	//SWITCH RX MODE
	TXLORA1 = 0;
	RXLORA1 = 1;

	//unread = 0;
	//tmax=0;
	while (DIO0LORA1 != 1){}
	/*	//sleep(1);
		tmax++;
		if (tmax>4000){
			unread = 1;
			break;
		}
	}

	if(unread==0)
	{*/
		//RESETING FLAG DIO0
		if((readRegLoRa(REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_RXDONE_MASK) == RFLR_IRQFLAGS_RXDONE){
			writeRegLora(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_RXDONE);
		}

		//STANDBY MODE
		writeRegLora(REG_LR_OPMODE, ((readRegLoRa(REG_LR_OPMODE) & RFLR_OPMODE_MASK) | RFLR_OPMODE_STANDBY));

		//SWITCH STANDBY MODE
		TXLORA1 = 0;
		RXLORA1 = 0;

		//RX NB BYTES
		packageLength =  readRegLoRa(REG_LR_RXNBBYTES);
		//printf("We received %d bytes\n",packageLength);

		writeRegLora(0x0D,readRegLoRa(0x10));

		//printf("\n\n");
		SS1 = 0;
		SPI1_Transmit(1, datasend, dataread);
		SPI1_Transmit(packageLength, datasend, dataread);
		//for (int i=0;i<packageLength;i++)
		//{
		//	datasend[0] = 0x00;
		//	SPI1_Transmit(1, datasend, dataread);
		//	printf("%c",dataread[0]);
		//}
		printf("\n\n");
		SS1 = 1;
		for (int i=0;i<(packageLength-1);i++)
		{
			printf("%c",dataread[i]);
		}

		last_pk_snr = readRegLoRa(REG_LR_PKTSNRVALUE); //0x19
		last_pk_rssi = readRegLoRa(REG_LR_PKTRSSIVALUE); //0x1A

		snr_dB = ((int)last_pk_snr / 4);

		if (last_pk_snr >= 0){
			rssi_dB = -139 + last_pk_rssi;
		}
		else{
			rssi_dB = -139 + last_pk_rssi + snr_dB;
		}
		printf("\n");
		printf("\nSNR = %03ddB  RSSI = %03ddB", snr_dB, rssi_dB);
/*	}s
	else{
		printf("\n");
		printf("TIMEOUT");
		unread = 0;
	}*/
	//vider la chaine
	for (int i=0;i<packageLength;i++)
	{
		dataread[i]='0';
	}
}

void sendTabLora1(unsigned char *messageTab)
{
	int length = 0;
	while(messageTab[length] != '\0'){
		length++;
	}
	SendPackageLoRa(messageTab, length);

}



