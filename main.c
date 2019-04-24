#include <agspCounter.h>
#include <agspMain.h>
#include <BLE.h>
#include <timer.h>
#include "system.h"
#include "GPS.h"
#include "LoRa.h"
#include "OLED.h"
#include "ALS.h"
#include "TMP2.h"
#include "HYGRO.h"
#include "BLE.h"

int main(void)
{
	int number = 0;
	char PackageReceive[256];

	SystemInit();

	printf("\r\n ---- Paul Program Smart Sensor ----\r\n\n\n"); // \xFF

	//initOLED();
	initALS();
	initGPS();
	initLoRa();
	initHYGRO();
	initAGSP();
	//initBLE();


	while (1)
	{
		if (TIMER1_FLAG){

			printf("SMART SENSOR N°%d", number);
			number++;
			printf("\n\n");

			//Phase SENSE :
			printf("PHASE SENSE");
			readGPS();
			readALS();
			readHYGRO();
			printf("\n\n");

			//Phase PROCESS :
			printf("PHASE PROCESS");
			cleanBufferMessage();
			processMessageHumidityHYGRO();
			processMessageALS();
			processMessageTemperatureHYGRO();
			processMessageGPS();
			printf("\n\n");

			//Phase SEND
			printf("PHASE SEND LORA");
			printBufferMessage();
			printf("\n");
			printDataSensorMessage();
			sendTabLora1(bufferMessage);
			printf("\n\n");
/*
			//Phase SEND
			printf("PHASE SEND BLE");
			//printBufferMessage();
			printDataSensorMessage();
			printf("\n\n");
			sendUartBLE(bufferMessage);
			readBLE();
*/
/*
			//Phase RECEIVE LORA
			printf("PHASE RECEIVE");
			ReceivePackageLoRa(PackageReceive);
			printf("\n\n");
*/

			//Phase SLEEP
			printf("PHASE SLEEP");
			printf("\n\n");


			printf("-----------------------------");
			printf("\n\n");

			//Remise à 0 du flag
			TIMER1_FLAG = 0;
		}

		//Phase SLEEP
		waitTC();
		_WFI();
		AGSP_counterLoop();

	}

	return 0;
}
