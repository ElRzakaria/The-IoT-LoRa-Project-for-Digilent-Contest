/*
	SX1272 LoRa for Linux SPI - YOCTO Dist
	Author : ZEL
	Original algorithms and functions : Laurent LATORRE(MEA POLYTECH MONTPELLIER) and KHUDUR ABDULLAH ALFARHAN and Ammar Zakaria (GitHub)
	
*/

#include <termios.h>
#include <stdio.h>
#include <unistd.h>//-
#include <sys/signal.h>
#include <sys/types.h>
#include <errno.h>
#include <string>
#include <iostream>
#include <fcntl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/stat.h> 
#include <signal.h>
#include <getopt.h>
#include "sx1272.h"

int interrupt_detected = 0;
int cpt=0,dd,dd1,dd2;
int ret1,ret2,ret;
char buf[10];
char buf2[10];
static __u32 mode = 0;
static __u32 mode_1=0;
static __u32 mode_2=0;
static __u8 bits = 8;
static __u32 speed = 1000000; // NORMALEMENT LORA_DEFAULT_SPI_FREQUENCY 250000
static __u16 delay;

struct spi_ioc_transfer xfer[2];

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void catch_function(int signo) 
{
	printf("\nInterrupt SIGINT detected. Terminating process...\n");
    interrupt_detected = 1;
	printf("Interrupt dans catch = %d\n",interrupt_detected);
}




int fspi1,fd_rx,fd_tx,fd_uart;
int val_gpio,ret_gpio;
char buf_gpio[16];

//************************ Initiation de La communication SPI ************************

int spi_init()
{
fspi1=open("/dev/spidev1.0",O_RDWR);
	
	if (fspi1 == -1){
		printf("Failed to open port fspi1\n");
		return -1;}

	ret = ioctl(fspi1, SPI_IOC_WR_MODE32, &mode);
	if (ret == -1)
		printf("can't set spi mode\n");

	ret = ioctl(fspi1, SPI_IOC_RD_MODE32, &mode);
	if (ret == -1)
		printf("can't get spi mode\n");

	ret = ioctl(fspi1, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		printf("can't set bits per word\n");

	ret = ioctl(fspi1, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
	printf("can't get bits per word\n");

	ret = ioctl(fspi1, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		printf("can't set max speed hz");

	ret = ioctl(fspi1, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		printf("can't get max speed hz");

    xfer[0].cs_change = 0; // Keep CS activated 
    xfer[0].delay_usecs = 0; //delay in us
    xfer[0].speed_hz = speed; //speed
    xfer[0].bits_per_word = 8; // bites per word 8

printf("Fin Init \n");
}

//************************ Export GPIOs RX(892) & TX(891) ************************

int gpioExport(int gpio)
{
    int fd;
    char buf[255];
    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd == -1) 
	{
	  printf("Failed to open %d port /sys/class/gpio/export \n",gpio);
	  return -1;
	}
    sprintf(buf, "%d", gpio); 
    write(fd, buf, strlen(buf));
    close(fd);
}

void gpioDirection(int gpio, int direction) // 1 for output, 0 for input
{
    int fd;
    char buf[255];
    sprintf(buf, "/sys/class/gpio/gpio%d/direction", gpio);
    fd = open(buf, O_WRONLY);

    if (direction)
    {
        write(fd, "out", 3);
    }
    else
    {
        write(fd, "in", 2);
    }
    close(fd);
}


void gpioSet(int fd,int gpio, int value) // 1 for High and 0 for LOw
{
    char buf[255];
    sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);
    fd = open(buf, O_WRONLY);
    sprintf(buf, "%d", value);
    write(fd, buf, 1);
    close(fd);
}

void gpioSetRXTX(int value) // 1 TRANSMISSION TX, 0 Reception RX, 2 BOTH OFF
{
    char buftx[255],bufrx[255];
    int fdrx, fdtx;
    sprintf(buftx, "/sys/class/gpio/gpio891/value");
    sprintf(bufrx, "/sys/class/gpio/gpio893/value");
    
    fdtx = open(buftx, O_WRONLY); fdrx = open(bufrx, O_WRONLY);

    if(value==0)
    {printf("Switch LoRa PINs Mode Reception\n");
     sprintf(buftx, "%d", 0); sprintf(bufrx, "%d", 1);}
    else if(value==1)
    {printf("Switch LoRa PINs Mode Transmission\n");
     sprintf(buftx, "%d",1); sprintf(bufrx, "%d", 0);}
    else
    {printf("Switch LoRa PINs Mode OFF RX=TX=Low\n");
    sprintf(buftx, "%d", 0); sprintf(bufrx, "%d", 0);}

    write(fdtx, buftx, 1);write(fdrx, bufrx, 1);
    close(fdtx); close(fdrx);
}

uint8_t gpioGet(int gpio)
{
  int fd;
  uint8_t value;
  char buf[255];
  sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);
  fd = open(buf, O_RDONLY);
  value=read(fd, buf, strlen(buf));
  close(fd);
  return buf[0]-48;
}

void resetlora(int value)
{
    char bufrst[255];
    int fdrst;
    sprintf(bufrst, "/sys/class/gpio/gpio892/value");//file RESET
    
    fdrst = open(bufrst, O_WRONLY);

	printf("Reset %d\n",value); sprintf(bufrst, "%d", value);
	write(fdrst, bufrst, 1);

	close(fdrst);/*
	SX1272 LoRa for Linux SPI - YOCTO Dist
	Author : ZEL
*/

}

void gpio_init(int gpio,int inout)
{
   gpioExport(gpio);
   gpioDirection(gpio,inout);
   printf("Initiation GPIO %d succeed \n",gpio);
}

void receive()
{
    int rx_length; 
    char buf[256];

   	rx_length = read(fd_uart, buf, 255);
	buf[rx_length] = '\0';
	printf("R: %s", buf);
}





uint8_t buft1[2]={0x00,0x00};
uint8_t buft12[2]={0x00,0x00};

uint8_t buftx1[1]={0x00};
uint8_t buftx12[1]={0x00};

struct spi_ioc_transfer tr12 {
		tr12.tx_buf = (unsigned long)buft1,
		tr12.rx_buf = (unsigned long)buft12,
		tr12.len = ARRAY_SIZE(buft1),
		tr12.delay_usecs = delay,
		tr12.speed_hz = speed,
		tr12.bits_per_word = 8};



struct spi_ioc_transfer trtx12 {
		trtx12.tx_buf = (unsigned long)buftx1,
		trtx12.rx_buf = (unsigned long)buftx12,
		trtx12.len = ARRAY_SIZE(buftx1),
		trtx12.delay_usecs = delay,
		trtx12.speed_hz = speed,
		trtx12.bits_per_word = 8};



void writeRegisterTEST(uint8_t add, uint8_t data)
{
   buft1[0]=add | WR_FLAG;
   buft1[1]=data;
   int cp = 0;
   while(buft12[1]!=buft1[1])
   { 
     ret2 = ioctl(fspi1, SPI_IOC_MESSAGE(1), &tr12);
	//printf(".");
     if (ret2 == -1) { printf("can't send spi message\n");}
}
   
    //printf("-%d Wr: %d - Tx : %02x %02x, Rx : %02x %02x\n",cp,ret2,buft1[0],buft1[1],buft12[0],buft12[1]);
}

void writeRegister_1(uint8_t data)
{
   buftx1[0]=data;
     ret2 = ioctl(fspi1, SPI_IOC_MESSAGE(1), &trtx12);
}

void writeRegister(uint8_t add, uint8_t data)
{
   buft1[0]=add | WR_FLAG;
   buft1[1]=data;
   //while(buft12[1]!=buft1[1])
   //{ 
     ret2 = ioctl(fspi1, SPI_IOC_MESSAGE(1), &tr12);
     if (ret2 == -1) { printf("can't send spi message\n");}
   //}
    //printf("Wr: %d - Tx : %02x %02x, Rx : %02x\n",ret2,buft1[0],buft1[1],buft12[1]);
}

uint8_t readRegister(uint8_t add)
{
   buft1[0]=add | RD_FLAG;
   buft1[1]=0x00;
   ret2 = ioctl(fspi1, SPI_IOC_MESSAGE(1), &tr12);
   if (ret2 == -1) { printf("can't send spi message\n");}
   //printf("Rd: %d - Reg : %02x, data : %02x\n",ret2,buft1[0],buft12[1]);
   return buft12[1];
}

//********************** MODE ************************************************
void Mode(uint8_t mode)
{
  writeRegisterTEST(REG_OP_MODE, mode);
}

//********************** CRC ************************************************
void CRC(uint8_t crc) // 1=ON, 0=OFF, default = 0;
{ 
  uint8_t crc_value;
printf("ENTER in crc function ...\n");
  
  if(crc) { crc_value=readRegister(REG_MODEM_CONFIG1) | 0x02 ;}
  else    { crc_value=readRegister(REG_MODEM_CONFIG1) & 0xFD ;}
  
  writeRegisterTEST(REG_MODEM_CONFIG1,crc_value);

  if(crc) { printf("CRC ON\n");}
  else    { printf("CRC OFF\n");}
}

//********************** Frequency Hoping ************************************************
void FrequencyHopping(uint8_t FH)
{
	if(FH == 1)
		{
		 printf("FrequencyHopping ON \n");
		 writeRegisterTEST(0x24, 0x11);
		}
	if(FH == 0)
		{
		 printf("FrequencyHopping OFF \n");
		 writeRegisterTEST(0x24, 0x00);
		}
}


//********************** Spreading Factor ************************************************

void SpreadingFactor(uint8_t SF)
{
Mode(LORA_STANDBY_MODE);

//To make the value of SF from 6 to 12
	if(SF < 6) {SF = 6;}
	if(SF > 12) {SF = 12;}

    if (SF == 6) 
    {
    writeRegisterTEST(0x31, 0xc5);
    writeRegisterTEST(0x37, 0x0c);
    } 
    else {
    writeRegisterTEST(0x31, 0xc3);
    writeRegisterTEST(0x37, 0x0a);
         }
  printf("SpreadingFactor %d \n",SF);
  writeRegisterTEST(0x1E, (readRegister(0x1E) & 0x0f) | ((SF << 4) & 0xf0));
}





void SendPackage(char *pack, uint8_t packlength)
{
   uint16_t i;
   uint8_t value;
   
   //Switch Lora to TX Mode
   printf("Setting Lora Mode in TX MODE...\n");
   writeRegister(REG_OP_MODE, LORA_TX_MODE);
}


//********************************** RECEIVING PACKAGES ***********************

void ReceivePackage(char *pack, uint8_t packlength)
{
  uint8_t lora_interrupt, packlocation,paul,paulo;

  printf("Configuration DIO0 pour RXDONE\n");
  writeRegisterTEST(REG_DIO_MAPPING1,((readRegister(REG_DIO_MAPPING1) & 0x3F) | 0x00)); // c'est les bits 7 et 6 du reg_dio_mapping qu'on doit programmer à 0x00 --> mask 0011.1111 | 0000.0000

  //Switch Lora to RX CONTINIOUS Mode
  printf("Setting Lora Mode in RX MODE...\n");
  Mode(LORA_RX_MODE); //RX MODE
  gpioSetRXTX(0);

  usleep(500);

  while(gpioGet(898)==0)
  {
  }; //wait for RXDONE

  printf("Message received, Setting Lora Mode in STANDBY MODE...\n");
  Mode(LORA_STANDBY_MODE); //RX MODE
  gpioSetRXTX(2); // MODE STANDBY SWITCH RX TX LORA (low)
  printf("Message received with %d bytes !\n",readRegister(REG_RX_NB_BYTES));
  printf("RX Current address %02x\n", readRegister(REG_FIFO_RX_CURRENT_ADDR));
  printf("RX Byte address %02x\n", readRegister(REG_FIFO_RX_BYTE_ADDR));
  printf("Set FIFO pointer to the start of last received message\n");
  writeRegister(REG_FIFO_ADDR_PTR,REG_FIFO_RX_CURRENT_ADDR); //(0x0D,0x10)

//Load FIFO TX POINTER
   
   writeRegister(0x11,0X87);
   writeRegister(0x12,0xFF);
   writeRegister(0x12,0x60);

   writeRegister(0x0d, readRegister(0x10));

   for(int i=0; i<packlength; i++)
   {
     *pack = readRegister(0x00);
     printf("%c",*pack);
     pack++;
   } 
  printf("\nLecture effectuée\n"); 

}

void beginTransmission(char *pack, uint8_t packlength)
{
  printf("Begin Transmission MODE\n");
  printf("Setting Lora Mode in StandBy MODE...\n");
  Mode(LORA_STANDBY_MODE);
  usleep(10000);
  
  printf("Set the NUmber of bytes to transmit\n");

  writeRegisterTEST(REG_PAYLOAD_LENGTH_LORA,packlength);

  printf("Configuration DIO0 pour TXDONE\n");
  writeRegisterTEST(REG_DIO_MAPPING1,((readRegister(REG_DIO_MAPPING1) & 0x3F) | 0x40)); // c'est les bits 7 et 6 du reg_dio_mapping qu'on doit programmer à 0x00 --> mask 0011.1111 | 0000.0000

 
  writeRegisterTEST(0x0D,0x80);
 printf("Writing FIFO...\n");
  //writeRegister_1(0x00|WR_FLAG);

//************************************************************************************* FILLING THE TX FIFO ********************************************************************
  for(int i=0; i<packlength; i++)
  {
    writeRegister(0x00,*pack);
	pack++;
  }

  printf("Setting Lora Mode in Tx MODE...\n");
  Mode(LORA_TX_MODE);
  usleep(10000);
  
  while(gpioGet(898)==0){}

  printf("DIO0 set\n");

  if((readRegister(REG_IRQ_FLAGS) & 0x08) == 0x08)
  {
    printf("TX done interrupt flag is set, now resetting...\n");
    writeRegister(REG_IRQ_FLAGS, 0x08);
    printf("Flag state ius now %02x \n", readRegister(REG_IRQ_FLAGS));
  }
}

void silentbeginTransmission(char *pack, uint8_t packlength)
{
  Mode(LORA_STANDBY_MODE);
  usleep(10000);

  writeRegisterTEST(REG_PAYLOAD_LENGTH_LORA,packlength);
  writeRegisterTEST(REG_DIO_MAPPING1,((readRegister(REG_DIO_MAPPING1) & 0x3F) | 0x40)); // c'est les bits 7 et 6 du reg_dio_mapping qu'on doit programmer à 0x00 --> mask 0011.1111 | 0000.0000

 
  writeRegisterTEST(0x0D,0x80);
  //writeRegister_1(0x00|WR_FLAG);

//************************************************************************************* FILLING THE TX FIFO ********************************************************************
  for(int i=0; i<packlength; i++)
  {
    writeRegister(0x00,*pack);
	pack++;
  }
  Mode(LORA_TX_MODE);
  usleep(10000);
  
  while(gpioGet(898)==0){}

  if((readRegister(REG_IRQ_FLAGS) & 0x08) == 0x08)
  {
    writeRegister(REG_IRQ_FLAGS, 0x08);
  }
}




int main(void){
//---------------------------------------------------
// Declarations
int rx_length;
char buf_uart[256];   					// Taille des messages recus par le  
int hie,fi;
int rd_mode1,rd_mode2;

struct termios tty;
int flags = O_RDWR | O_NOCTTY | O_NDELAY/*| O_NONBLOCK*/;

uint8_t buf[2]={0x00,0x00};
uint8_t buf2[2]={0x00,0x00};
uint8_t LoRa_Vers=0x22;
int et0,et1,et2,et3,et4,et5=0;

struct spi_ioc_transfer tr2 ={};
		tr2.tx_buf = (unsigned long)buf;
		tr2.rx_buf = (unsigned long)buf2;
		tr2.len = ARRAY_SIZE(buf);
		tr2.delay_usecs = delay;
		tr2.speed_hz = speed;
		tr2.bits_per_word = 8;
    
    



//----------------Open file spi, TX = 891 & RX = 893, RESET=892 and UART for pcCAM5------------------------------
    fspi1=open("/dev/spidev1.0",O_RDWR);
    fd_tx=open("/sys/class/gpio891/gpio/value", O_WRONLY);
    fd_rx=open("/sys/class/gpio893/gpio/value", O_WRONLY);
	fd_uart = open("/dev/ttyUL0", flags);
	if (fd_uart == -1){
    	printf("Failed to open port\n");
    	return -1;
    }

	tcgetattr(fd_uart, &tty);
    tty.c_oflag = 0;
    tty.c_iflag = 0;//tty.c_iflag | IXON | ICRNL; // donc = 0
    tty.c_cflag = tty.c_cflag | CS8 | CREAD | B9600; //B9600; //z B115200;
    tty.c_cc[VMIN]=1;
    tty.c_cc[VTIME]=0; 
    tty.c_lflag &= ~ECHO;
    if (tcsetattr (fd_uart, TCSANOW, &tty) != 0)
    {
        fprintf (stderr, "error 'fd_uart' %d from tcsetattr", errno);
        return -1;
    }

    tcflush(fd_uart, TCIOFLUSH);
    printf("UART ok -- Connection ok\n");



//********************INITIALISATION TX & RX*****************
//gpioGet(898)
gpio_init(891,1);//TX OUT
gpio_init(893,1);//RX OUT
gpio_init(892,1);//RESET (RTX) OUT
gpio_init(898,0);//DIO0 OUT
gpioSetRXTX(2);
resetlora(1);

sleep(1);
resetlora(0);

//********************INITIALISATION DU SPI*****************
printf("Initialisation spi...\n");
spi_init();

//********************Lecture Version LORA*****************
printf("\nReading Lora Mode...\n");
readRegister(REG_VERSION);

//********************Desactivation FSK MODE*****************
printf("Setting Lora Mode in FSK SLEEP MODE...\n");
writeRegisterTEST(REG_OP_MODE, FSK_SLEEP_MODE);


//********************INITIALISATION LORA MODE*****************

printf("Setting Lora Mode in SLEEP MODE...\n");//LORA SLEEP MODE
Mode(LORA_SLEEP_MODE);
printf("Setting Lora Mode in STANDBY MODE...\n");//LORA STAND BY MODE
Mode(LORA_STANDBY_MODE);

usleep(50);

printf("Configuration DIO0 pour RXDONE\n");
writeRegisterTEST(REG_DIO_MAPPING1,((readRegister(REG_DIO_MAPPING1) & 0x3F) | 0x00)); // c'est les bits 7 et 6 du reg_dio_mapping qu'on doit programmer à 0x00 --> mask 0011.1111 | 0000.0000

printf("Setting Lora CRC ON...\n");
CRC(1);
printf("Setting Lora Frequency Hoping ON...\n");
FrequencyHopping(1);
printf("Setting Lora SPREADING FACTOR 12...\n");
SpreadingFactor(12);
usleep(100);

//gpioSetRXTX(0);
//printf("Setting Lora Mode in RX MODE...\n");
//Mode(LORA_RX_MODE); //RX MODE


printf("Mode lora %02x \n",readRegister(0x01));

   /*writeRegister(REG_IRQ_FLAGS_MASK,0X87);
   writeRegister(REG_IRQ_FLAGS,0xFF);
   writeRegister(REG_FIFO_ADDR_PTR,readRegister(REG_FIFO_RX_CURRENT_ADDR));

*/




//********************BOUCLE INFINIE*****************
printf("Ctrl+C pour arreter le process \n");
char p[]="***************";
char q[]="Holla\r\n";
char r[256];
char rq[256];
int l=0;
//sprintf(q, "Holla Senior%d\r\n", l);
sprintf(q, "012345678901234567890123456789%d\r\n", l++);


//beginTransmission(q,strlen(q));


while(interrupt_detected!=1)
{

//	fonction emission de donnee de la pcam5

rx_length = read(fd_uart, buf_uart, 50);
if(rx_length>0)
{
printf("\n\nMouvement detetcted \n");
usleep(500);
printf("%s",buf_uart);
silentbeginTransmission(buf_uart,strlen(buf_uart));
//memset(buf_uart,0,sizeof(buf_uart));
rx_length=0;
}


}
printf("Sorti de while\n");
  	
//-----------------------------------------------------
// Clean-up
printf("Clean-up\n");
close(fd_tx);
close(fd_rx);
close(fd_uart);
close(fspi1);
return 0;
}






