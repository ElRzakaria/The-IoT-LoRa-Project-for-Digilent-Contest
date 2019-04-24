/**
 * GPS library
 * author: Paul Leloup
 * update: 15-10-2018
 */

#define LENTHTRAM 300
#define LENTHBUFFERUART2 1024

int i,x,end,step;
unsigned char buffer[LENTHBUFFERUART2];
unsigned char bufferTram[LENTHTRAM];

int initGPS(void);
void readGPS(void);
int tramTabRecupGPS(void);
int printTramGPS(void);
int processMessageGPS(void);
int bufferInit(void);
int processMessageGPS(void);
int lengthMessageGPS();
int printMessageGPS(void);
int printIfNotNull(unsigned char c, int j);
int cleanTramGPS(void);

