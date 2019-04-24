/**
 * TIMER library
 * author: Paul Leloup
 * update: 11-01-2018
 */

#include <timer.h>
#include "system.h"

#pragma message("Compiling functions for TIMER")

void ClockInit(void)
{

	TIMER1.PE = 0;
	TIMER1.UIE = 1;
	TIMER1.PSC = (2*SYSCLK-CLOCKS_PER_SEC)/(2*CLOCKS_PER_SEC);
	TIMER1.ARR = PERIOD_TIMER_1-1; // 1000 ms = wake up every second
	TIMER1.PE = 1;

	NVIC_SET_ENABLE(NVIC_TIMER1);

}


void sleep(unsigned long int n) {
        /* boucle vide parcourue (n * 100000) fois*/
        int i = 0;
        unsigned long int max = n * 1000;
        do {
                /* Faire qqch de stupide qui prend du temps */
                i++;
        }
        while(i <= max);
}

void TIMER1_IRQHandler(void)
{
	if (TIMER1.UIF)
	{
		TIMER1.UIF = 0;

		TIMER1_FLAG = 1;
	}
}
