/**
 * SPI library
 * author: Guillaume Patrigeon -- autogenerated
 * update: 08-01-2019
 */

#include "system.h"


//--------------------------------------------------------
#if defined(SPI1) && defined(_USE_SPI1)
#pragma message("Compiling functions for SPI1")

static volatile unsigned char* SPI1_TxBuffer;
static volatile unsigned char* SPI1_RxBuffer;
static volatile int SPI1_Count;


void SPI1_Init(void)
{
	SPI1.CPHA = 0;
	SPI1.CPOL = 0;

	// SPI1.RXBFIE = 1;
}


void SPI1_Transmit(int size, const unsigned char* txbuffer, const unsigned char* rxbuffer)
{
	// SPI1_TxBuffer = (unsigned char*)txbuffer;
	// SPI1_RxBuffer = (unsigned char*)rxbuffer;
	// SPI1_Count = size;

	// SPI1.TXBEIE = 1;
	// while (SPI1_Count);// _WFI();
	// while (!SPI1.TC);

	unsigned char* tx = (unsigned char*)txbuffer;
	unsigned char* rx = (unsigned char*)rxbuffer;

	while (size--)
	{
		MONITOR.CR = MONITOR_CR_STOP;
		while (!SPI1.TXBE);
		MONITOR.CR = MONITOR_CR_START;

		SPI1.DATA = *(tx++);

		MONITOR.CR = MONITOR_CR_STOP;
		while (!SPI1.RXBF);
		MONITOR.CR = MONITOR_CR_START;

		*(rx)++ = SPI1.DATA;
	}
}


void SPI1_IRQHandler(void)
{
	if (SPI1.TXBE && SPI1.TXBEIE)
	{
		if (SPI1_Count)
		{
			SPI1.DATA = *(SPI1_TxBuffer++);
			SPI1_Count--;
			LED1 = 1;
		}
		else
		{
			SPI1.TXBEIE = 0;
			LED2 = 1;
		}
	}

	if (SPI1.RXBF)
		*(SPI1_RxBuffer)++ = SPI1.DATA;
}

#endif



//--------------------------------------------------------
#if defined(SPI2) && defined(_USE_SPI2)
#pragma message("Compiling functions for SPI2")

static volatile unsigned char* SPI2_TxBuffer;
static volatile unsigned char* SPI2_RxBuffer;
static volatile int SPI2_Count;


void SPI2_Init(void)
{
	SPI2.CPHA = 0;
	SPI2.CPOL = 0;

	// SPI2.RXBFIE = 1;
}


void SPI2_Transmit(int size, const unsigned char* txbuffer, const unsigned char* rxbuffer)
{
	// SPI2_TxBuffer = (unsigned char*)txbuffer;
	// SPI2_RxBuffer = (unsigned char*)rxbuffer;
	// SPI2_Count = size;

	// SPI2.TXBEIE = 1;
	// while (SPI2_Count);// _WFI();
	// while (!SPI2.TC);

	unsigned char* tx = (unsigned char*)txbuffer;
	unsigned char* rx = (unsigned char*)rxbuffer;

	while (size--)
	{
		MONITOR.CR = MONITOR_CR_STOP;
		while (!SPI2.TXBE);
		MONITOR.CR = MONITOR_CR_START;

		SPI2.DATA = *(tx++);

		MONITOR.CR = MONITOR_CR_STOP;
		while (!SPI2.RXBF);
		MONITOR.CR = MONITOR_CR_START;

		*(rx)++ = SPI2.DATA;
	}
}


void SPI2_IRQHandler(void)
{
	if (SPI2.TXBE && SPI2.TXBEIE)
	{
		if (SPI2_Count)
		{
			SPI2.DATA = *(SPI2_TxBuffer++);
			SPI2_Count--;
			LED1 = 1;
		}
		else
		{
			SPI2.TXBEIE = 0;
			LED2 = 1;
		}
	}

	if (SPI2.RXBF)
		*(SPI2_RxBuffer)++ = SPI2.DATA;
}

#endif



//--------------------------------------------------------
#if defined(SPI3) && defined(_USE_SPI3)
#pragma message("Compiling functions for SPI3")

static volatile unsigned char* SPI3_TxBuffer;
static volatile unsigned char* SPI3_RxBuffer;
static volatile int SPI3_Count;


void SPI3_Init(void)
{
	SPI3.CPHA = 0;
	SPI3.CPOL = 0;

	// SPI3.RXBFIE = 1;
}


void SPI3_Transmit(int size, const unsigned char* txbuffer, const unsigned char* rxbuffer)
{
	// SPI3_TxBuffer = (unsigned char*)txbuffer;
	// SPI3_RxBuffer = (unsigned char*)rxbuffer;
	// SPI3_Count = size;

	// SPI3.TXBEIE = 1;
	// while (SPI3_Count);// _WFI();
	// while (!SPI3.TC);

	unsigned char* tx = (unsigned char*)txbuffer;
	unsigned char* rx = (unsigned char*)rxbuffer;

	while (size--)
	{
		MONITOR.CR = MONITOR_CR_STOP;
		while (!SPI3.TXBE);
		MONITOR.CR = MONITOR_CR_START;

		SPI3.DATA = *(tx++);

		MONITOR.CR = MONITOR_CR_STOP;
		while (!SPI3.RXBF);
		MONITOR.CR = MONITOR_CR_START;

		*(rx)++ = SPI3.DATA;
	}
}


void SPI3_IRQHandler(void)
{
	if (SPI3.TXBE && SPI3.TXBEIE)
	{
		if (SPI3_Count)
		{
			SPI3.DATA = *(SPI3_TxBuffer++);
			SPI3_Count--;
			LED1 = 1;
		}
		else
		{
			SPI3.TXBEIE = 0;
			LED2 = 1;
		}
	}

	if (SPI3.RXBF)
		*(SPI3_RxBuffer)++ = SPI3.DATA;
}

#endif



//--------------------------------------------------------
#if defined(SPI4) && defined(_USE_SPI4)
#pragma message("Compiling functions for SPI4")

static volatile unsigned char* SPI4_TxBuffer;
static volatile unsigned char* SPI4_RxBuffer;
static volatile int SPI4_Count;


void SPI4_Init(void)
{
	SPI4.CPHA = 0;
	SPI4.CPOL = 0;

	// SPI4.RXBFIE = 1;
}


void SPI4_Transmit(int size, const unsigned char* txbuffer, const unsigned char* rxbuffer)
{
	// SPI4_TxBuffer = (unsigned char*)txbuffer;
	// SPI4_RxBuffer = (unsigned char*)rxbuffer;
	// SPI4_Count = size;

	// SPI4.TXBEIE = 1;
	// while (SPI4_Count);// _WFI();
	// while (!SPI4.TC);

	unsigned char* tx = (unsigned char*)txbuffer;
	unsigned char* rx = (unsigned char*)rxbuffer;

	while (size--)
	{
		MONITOR.CR = MONITOR_CR_STOP;
		while (!SPI4.TXBE);
		MONITOR.CR = MONITOR_CR_START;

		SPI4.DATA = *(tx++);

		MONITOR.CR = MONITOR_CR_STOP;
		while (!SPI4.RXBF);
		MONITOR.CR = MONITOR_CR_START;

		*(rx)++ = SPI4.DATA;
	}
}


void SPI4_IRQHandler(void)
{
	if (SPI4.TXBE && SPI4.TXBEIE)
	{
		if (SPI4_Count)
		{
			SPI4.DATA = *(SPI4_TxBuffer++);
			SPI4_Count--;
			LED1 = 1;
		}
		else
		{
			SPI4.TXBEIE = 0;
			LED2 = 1;
		}
	}

	if (SPI4.RXBF)
		*(SPI4_RxBuffer)++ = SPI4.DATA;
}

#endif



