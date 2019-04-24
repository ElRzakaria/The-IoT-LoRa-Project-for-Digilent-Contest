/**
 *
 * pmodBLE_client_bean
 * Light Blue Bean
 * reads characteristics from Light Blue Bean
 *
 */

#include "system.h"
#include "BLE.h"

#pragma message("Compiling functions for BLE")
int fd;

void initBLE(void)
{
	RESET





	= 0;
	sleep(1000); // Reset au moins 55ms
	RESET = 1;
	sleep(20);

	// Receive a frame
	sleep(1000);
	readBLE();

	// Command mode
	sendUartBLE("$$$");
	readBLE();

	// Echo
	sendUartBLE("+\r");
	readBLE();

	// Configuration
	sendUartBLE("SS,C0\r");
	readBLE();

	//REBOOT
	sendUartBLE("R,1\r");
	readBLE();

	// Command Mode
	sendUartBLE("$$$");
	readBLE();

	// Informations
	sendUartBLE("D\r");
	readBLE();


	// Connect to PMOD BLE TEST CONSO
	sendUartBLE("C,0,D88039F78710\r");
	sleep(5000);
	readBLE();
/*
	// Get remote device name
	sendUartBLE("GNR\r");
	readBLE();

	// Get current connection status
	sendUartBLE("GK\r");
	readBLE();

	// Read RSSI value of connected device
	sendUartBLE("M\r");
	readBLE();

	// Read Version
	sendUartBLE("V\r");
	readBLE();

	// Client mode
	sendUartBLE("CI\r");
	readBLE();

	// Lists the available client services and their characteristics
	sendUartBLE("LC\r");
	readBLE();
*/
}

void readBLE(void)
{
	sleep(1000);
	while(UART3_IsRxNotEmpty())
	{
	printf("%c",UART3_Read());
	}
	printf("\n\n");
}

void readXTimeBLE(int time)
{
	for(int m = 0 ; m < time ; m++)
	{
	printf("%c",UART3_Read());
	}
}

void sendUartBLE(char *buf)
{
	int length = 0;

	while(buf[length] != '\0'){
		length++;
	}

	for(int m = 0 ; m < length ; m++)
	{
		printf("%c",buf[m]);
		UART3_Send(buf[m]);
	}
	printf("\n");

}

void BLE (char *buf)
{
	sendUartBLE(buf);
	readBLE();
}
int hex2dec (char * tab)
{
    int           x = 0;
    unsigned char c = 0;

    while (*tab != '\0')
    {
        if      (*tab >= '0' && *tab <= '9') c = *tab - '0';
        else if (*tab >= 'a' && *tab <= 'f') c = *tab - 'a' + 10;
        else if (*tab >= 'A' && *tab <= 'F') c = *tab - 'A' + 10;
        else break;

        x = x * 16 + c;

        tab++;
    }

    return x;
}

/*
    while(1)
    {
        strcpy(buf,"CHR,0037\r"); // Reads the content of a characteristic
        sendUart(buf);
        receiveUart(buf);
 //       value = hex2dec(buf);
        printf("ACC X = %s\n",buf);

        strcpy(buf,"CHR,003B\r"); // Reads the content of a characteristic
        sendUart(buf);
        receiveUart(buf);
//        value = hex2dec(buf);
        printf("ACC Y = %s\n",buf);

        strcpy(buf,"CHR,003F\r"); // Reads the content of a characteristic
        sendUart(buf);
        receiveUart(buf);
//        value = hex2dec(buf);
        printf("ACC Z = %s\n",buf);

        strcpy(buf,"CHR,0043\r"); // Reads the content of a characteristic
        sendUart(buf);
        receiveUart(buf);
        value = hex2dec(buf);
        printf("Temperature = %d °C\n",value);

        strcpy(buf,"CHR,0048\r"); //Reads the content of a characteristic
        sendUart(buf);
        receiveUart(buf);
        value = hex2dec(buf);
        printf("Battery Level = %d %%\n",value);
    }

    strcpy(buf,"---\r");
    sendUart(buf);
    receiveUart(buf);

//-----------------------------------------------------
    close(fd);
    return 0;
}



*/

