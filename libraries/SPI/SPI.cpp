/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */
#include <Arduino.h>
#include "_spi.h"
#include "SPI.h"

#define SPI_WRITE_TX(spi, u32TxData) 	SPI_WRITE_TX0(spi, u32TxData);
#define SPI_READ_RX(spi)                SPI_READ_RX0(spi)
				

SPIClass::SPIClass(SPI_T *_spi) :
	spi(_spi)
{		
	  //initCb();  	  
}

void SPIClass::begin() {
	
	if(init_flag==0); //init();
	
	/* Unlock protected registers */
	SYS_UnlockReg();	

	/* Enable IP clock */       
	CLK_EnableModuleClock(SPI1_MODULE);    	
   	
	/* Select IP clock source and clock divider */
	CLK_SetModuleClock(SPI1_MODULE,CLK_CLKSEL1_SPI1_S_HCLK,MODULE_NoMsk);
	SYS->GPC_MFP |= SYS_GPC_MFP_PC11_SPI1_MOSI0;
	SYS->GPA_MFP |= SYS_GPA_MFPH_GPA10_MFP_SPI1_MISO0 | SYS_GPA_MFPH_GPA11_MFP_SPI1_CLK;
	SYS->ALT_MFP |= SYS_ALT_MFP_PC11_SPI1_MOSI0 | SYS_ALT_MFP_PA11_SPI1_CLK | SYS_ALT_MFP_PA10_SPI1_MISO0;

	/* Lock protected registers */
	SYS_LockReg();
			  
	/* Configure as a master, clock idle low, falling clock edge Tx, rising edge Rx and 8-bit transaction */
	/* Set IP clock divider. SPI clock rate = 12MHz */
	SPI_Open(spi, SPI_MASTER, SPI_MODE_0, 8, 6000000);
	//SPI_EnableFIFO(spi,2,2); //ouki
	setBitOrder(SS, MSBFIRST);
}

void SPIClass::end() {
	SPI_Close(spi);
}

void SPIClass::usingInterrupt(uint8_t interruptNumber)
{
}

void SPIClass::beginTransaction(uint8_t pin, SPISettings settings)
{
        if(init_flag==0);// init();
	/* Configure as a master, clock idle low, falling clock edge Tx, rising edge Rx and 8-bit transaction */
	/* Set IP clock divider. SPI clock rate = 4MHz */  	
	SPI_Open(spi, SPI_MASTER, settings.datmode, 8, settings.clock);	
	//SPI_EnableFIFO(spi,12,12);	
	setBitOrder(SS, settings.border);

//#if defined(__M451__) | defined(__NANO100__)	
	SPI_ClearRxFIFO(spi);
	SPI_TRIGGER(spi);
//#endif
}

void SPIClass::endTransaction(void)
{
	
}

void SPIClass::setBitOrder(uint8_t _pin, BitOrder _bitOrder) {	
	if(_bitOrder==LSBFIRST)
		SPI_SET_LSB_FIRST(spi);
	else
		SPI_SET_MSB_FIRST(spi);
}

void SPIClass::setDataMode(uint8_t _pin, uint8_t _mode) {	
	spi->CNTRL = (spi->CNTRL & ~SPI_MODE_Msk) | _mode;
}

void SPIClass::setClockDivider(uint8_t _pin, uint8_t _divider) {
	#if 0
	_divider=((_divider+1)>>1)-1;
	spi->DIVIDER = (spi->DIVIDER & ~0xffff) | _divider;
	#endif
}

byte SPIClass::transfer(byte _pin, uint8_t _data, SPITransferMode _mode) {	
	uint32_t rdata;
	SPI_WRITE_TX(spi, _data);
	SPI_TRIGGER(spi);
	while(SPI_IS_BUSY(spi));
	return (SPI_READ_RX(spi) & 0xff);
}

void SPIClass::read(void *buf,int len) {	
	u8 * p = (u8 *)buf;
	int i;
	for(i=0; i<len; i++){
		SPI_WRITE_TX(spi, 0xff);
		SPI_TRIGGER(spi);
		while(SPI_IS_BUSY(spi));
		p[i] = SPI_READ_RX(spi) & 0xff;
	}
}

void SPIClass::attachInterrupt(void) {
	// Should be enableInterrupt()
}

void SPIClass::detachInterrupt(void) {
	// Should be disableInterrupt()
}

#if SPI_MAX_COUNT > 0
SPIClass SPI(SPI1);
#endif

