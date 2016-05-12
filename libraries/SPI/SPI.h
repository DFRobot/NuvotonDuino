/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include "Arduino.h"
#include "SPI.h"

#define BOARD_SPI_DEFAULT_SS 43

#define SPI_MODE0 SPI_MODE_0
#define SPI_MODE1 SPI_MODE_1
#define SPI_MODE2 SPI_MODE_2
#define SPI_MODE3 SPI_MODE_3
#define SPI_MODE_Msk (SPI_MODE_0|SPI_MODE_1|SPI_MODE_2|SPI_MODE_3)


enum SPITransferMode {
	SPI_CONTINUE,
	SPI_LAST
};

class SPISettings {
public:
	SPISettings(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) {
		/*if (__builtin_constant_p(clock)) {
			init_AlwaysInline(clock, bitOrder, dataMode);
		} else {
			init_MightInline(clock, bitOrder, dataMode);
		}*/
		this->clock = clock;
		this->datmode = dataMode;
		this->border = bitOrder;
	}
	SPISettings() { init_AlwaysInline(4000000, MSBFIRST, SPI_MODE0); }
private:
	void init_MightInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) {
		init_AlwaysInline(clock, bitOrder, dataMode);
	}
	void init_AlwaysInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) __attribute__((__always_inline__)) {
		border = bitOrder;
		clock = clock;
		datmode = dataMode;
	}
	uint32_t datmode;
	uint32_t clock;
	BitOrder border;
	friend class SPIClass;
};

class SPIClass {
  public:
	SPIClass(SPI_T *_spi);

	byte transfer(uint8_t _data, SPITransferMode _mode = SPI_LAST) { return transfer(SS, _data, _mode); }
	void transfer(byte _pin, void *_buf, size_t _count, SPITransferMode _mode = SPI_LAST);
	
	byte transfer(byte _channel, uint8_t _data, SPITransferMode _mode = SPI_LAST);
	void transfer(void *_buf, size_t _count, SPITransferMode _mode = SPI_LAST) { transfer(BOARD_SPI_DEFAULT_SS, _buf, _count, _mode); }

	// Transaction Functions
	void usingInterrupt(uint8_t interruptNumber);
	void beginTransaction(SPISettings settings) { beginTransaction(BOARD_SPI_DEFAULT_SS, settings); }
	void beginTransaction(uint8_t pin, SPISettings settings);
	void endTransaction(void);
	void read(void *buf,int len);
	// SPI Configuration methods
	void attachInterrupt(void);
	void detachInterrupt(void);

	void begin(void);
	void end(void);

	// Attach/Detach pin to/from SPI controller
	//void begin(uint8_t _pin);
	//void end(uint8_t _pin);

	// These methods sets a parameter on a single pin
	void setBitOrder(uint8_t _pin, BitOrder);
	void setDataMode(uint8_t _pin, uint8_t);
	void setClockDivider(uint8_t _pin, uint8_t);

	// These methods sets the same parameters but on default pin BOARD_SPI_DEFAULT_SS
	void setBitOrder(BitOrder _order) { setBitOrder(SS, _order); };
	void setDataMode(uint8_t _mode) { setDataMode(SS, _mode); };
	void setClockDivider(uint8_t _div) { setClockDivider(SS, _div); };

  private:
	SPI_T *spi;
	uint32_t module;
	uint32_t clksel;
	IRQn_Type id;
	//BitOrder bitOrder[SPI_CHANNELS_NUM];
	//uint32_t divider[SPI_CHANNELS_NUM];
	//uint32_t mode[SPI_CHANNELS_NUM];
	void (*initCb)(void);
};

#if SPI_MAX_COUNT > 0
	extern SPIClass SPI;
#endif

#endif

