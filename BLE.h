/**
 * BLE library
 * author: Paul Leloup
 * update: 16-10-2018
 */

#define RESET GPIOD.ODRbits.P1 //A MODIFIER

void initBLE(void);
void readBLE(void);
void readXTimeBLE(int time);
void sendUartBLE(char *buf);
int hex2dec (char * tab);



/*
void gpioSet(int gpio, int value);
void sendUart(char *buf);
void receiveUart(char *buf);
int hex2dec (char * tab);
*/
