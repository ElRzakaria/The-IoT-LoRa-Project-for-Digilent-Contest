/**
 * TIMER library
 * author: Paul Leloup
 * update: 11-01-2018
 */

#define PERIOD_TIMER_1 5000 // Periode en ms

int TIMER1_FLAG ;

void ClockInit(void);

void TIMER1_IRQHandler(void);

void sleep(unsigned long int n);
