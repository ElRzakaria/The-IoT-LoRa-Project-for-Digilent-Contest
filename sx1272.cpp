/*
	SX1272 LoRa for Linux SPI - YOCTO Dist
	Author : ZEL
*/

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <errno.h>
#include <string>
#include <string.h>
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

struct spi_ioc_transfer tr2 ={};
		tr2.tx_buf = (unsigned long)buf;
		tr2.rx_buf = (unsigned long)buf2;
		tr2.len = ARRAY_SIZE(buf);
		tr2.delay_usecs = delay;
		tr2.speed_hz = speed;
		tr2.bits_per_word = 8;
int ret,fspi2;
uint8_t buf[2]={0x00,0x00};
uint8_t buf2[2]={0x00,0x00};
fspi2=open("/dev/spidev2.0",O_RDWR);
/*
void writeRegister(uint8_t address, uint8_t data)
{
    buf[0]=address | WR_FLAG;
    buf[1]=data;
    ret = ioctl(fspi2, SPI_IOC_MESSAGE(1), &tr2);
    if (ret == -1) { printf("can't send spi message\n");}
    printf("ioctl : %d - Tx : %02x %02x, Rx : %02x %02x\n",ret,buf[0],buf[1],buf2[0],buf2[1]);
}

uint8_t readRegister(uint8_t address, uint8_t data)
{
    buf[0]=address | RD_FLAG;
    buf[1]=data;
    ret = ioctl(fspi2, SPI_IOC_MESSAGE(1), &tr2);
    if (ret == -1) { printf("can't send spi message\n");}
    printf("ioctl : %d - Tx : %02x %02x, Rx : %02x %02x\n",ret,buf[0],buf[1],buf2[0],buf2[1]);
    return buf2[1];
}
*/

int interrupt_detected = 0;
int cpt=0,dd,dd1,dd2;
int ret1,ret2,ret;
char buf[10];
char buf2[10];
static __u32 mode = 0;
static __u32 mode_1=0;
static __u32 mode_2=0;
static __u8 bits = 8;
static __u32 speed = 1000000;
static __u16 delay;


