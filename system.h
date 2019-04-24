#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include <armv6m.h>
#include <arch.h>

#include <ascii.h>
#include <ansi.h>
#include <print.h>


#ifndef bool
typedef enum {false, true} bool;
#endif

typedef unsigned int size_t;
typedef int clock_t;

#define _USE_SPI1
#define _USE_SPI2
#define _USE_I2C1
//#define _USE_I2C2

#include "spi.h"
#include "i2c.h"
#include "uart.settings.h"
#include "uart.h"
#include "timer.h"
#include "smart_sensor.h"

#define BTN GPIOA.IDRbits.P0
#define LED1 GPIOA.ODRbits.P1
#define LED2 GPIOA.ODRbits.P2

#define SYSCLK                  12000000
#define CLOCKS_PER_SEC          1000

#ifdef _USE_UART1
#define printc(c)               UART1_Send(c)
#define printf(...)             print(UART1_Send, __VA_ARGS__)
#define waitTC()                while (!UART1.TC)
#else
#define printc(c)
#define printf(...)
#define waitTC()
#endif

#define printf2(...)             print(UART2_Send, __VA_ARGS__)
#define printf3(...)             print(UART3_Send, __VA_ARGS__)


#define _abs(x) ((x) < 0 ? -(x) : (x))

void SystemInit(void);


int main(void);


#define timer_1_init(...)
#define timer_1_enable(...)
#define timer_1_reset(...)

#define timer_2_init(...)
#define timer_2_enable(...)
#define timer_2_reset(...)


#endif
