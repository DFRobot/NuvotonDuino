/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file HardwareSerial.cpp
 * @brief Wirish serial port implementation.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"



ring_buffer rx_buffer1 = { { 0 }, 0, 0};  /* for UAR1_IRQ */
HardwareSerial Serial1(UART0, &rx_buffer1);

#if defined(__NUC123__)
#elif defined(__BlunoM0__)
HardwareSerial &Serial = Serial1;
#endif

ring_buffer rx_buffer2 = { { 0 }, 0, 0};  /* for UAR1_IRQ */

HardwareSerial Serial2(UART1, &rx_buffer2);


#ifdef __cplusplus
extern "C"
{
#endif
#if 1
void UART0_IRQHandler(void)
{    
	while(UART_GET_INT_FLAG(UART0,UART_IER_RDA_IEN_Msk))
	{
		int i = (unsigned int)(rx_buffer1.head + 1) % SERIAL_BUFFER_SIZE;
  	if (i != rx_buffer1.tail) {
    	rx_buffer1.buffer[rx_buffer1.head] = UART0->RBR;
    	rx_buffer1.head = i;
  	}
	}
}
void UART1_IRQHandler(void)
{    
	while(UART_GET_INT_FLAG(UART1,UART_IER_RDA_IEN_Msk))
	{
		int i = (unsigned int)(rx_buffer2.head + 1) % SERIAL_BUFFER_SIZE;
  	if (i != rx_buffer2.tail) {
    	rx_buffer2.buffer[rx_buffer2.head] = UART1->RBR;
    	rx_buffer2.head = i;
  	}
	}
}

#endif
void serialEvent() __attribute__((weak));
#ifdef __cplusplus
}
#endif


void serialEventRun(void)
{
    if (Serial1.available()) serialEvent();
}



HardwareSerial::HardwareSerial(UART_T *uart_device, ring_buffer *rx_buffer) {
    this->uart_device = uart_device;
    this->_rx_buffer = rx_buffer;
}

/*
 * Set up/tear down
 */

void HardwareSerial::begin(uint32_t baud) {
	SYS_UnlockReg();
	if(uart_device == UART0){
		CLK_EnableModuleClock(UART0_MODULE);
		CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
		/* Set GPB multi-function pins for UART0 RXD and TXD */
		SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC4_Msk | SYS_GPC_MFP_PC5_Msk);
		SYS->GPC_MFP |= (SYS_GPC_MFP_PC4_UART0_RXD | SYS_GPC_MFP_PC5_UART0_TXD);
		SYS->ALT_MFP |= SYS_ALT_MFP_PC4_UART0_RXD | SYS_ALT_MFP_PC5_UART0_TXD;
		
		SYS_LockReg();
		SYS_ResetModule(UART0_RST);
	    /* Enable Interrupt */
	    UART_ENABLE_INT(UART0, UART_IER_RDA_IEN_Msk);

	    /* Configure UART and set UART Baudrate */
	    UART_Open(uart_device, baud);
		NVIC_SetPriority(UART0_IRQn, 1);//后面改为1
	    NVIC_EnableIRQ(UART0_IRQn);
	}else if(uart_device == UART1){
		CLK_EnableModuleClock(UART1_MODULE);
		CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
		/* Set GPB multi-function pins for UART0 RXD and TXD */
		SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB4_Msk | SYS_GPB_MFP_PB5_Msk);
		SYS->GPB_MFP |= (SYS_GPB_MFP_PB4_UART1_RXD | SYS_GPB_MFP_PB5_UART1_TXD);
	
		SYS_LockReg();
		SYS_ResetModule(UART1_RST);
	    /* Enable Interrupt */
	    UART_ENABLE_INT(UART1, UART_IER_RDA_IEN_Msk);

	    /* Configure UART and set UART Baudrate */
	    UART_Open(uart_device, baud);
		NVIC_SetPriority(UART1_IRQn, 1);//后面改为1
	    NVIC_EnableIRQ(UART1_IRQn);
	}else{
		SYS_LockReg();
	}
}

void HardwareSerial::end(void) {
    UART_Close(uart_device);
}

/*
 * I/O
 */

int HardwareSerial::read(void) {        
  // if the head isn't ahead of the tail, we don't have any characters
  if (_rx_buffer->head == _rx_buffer->tail) {
    return -1;
  } else {
    unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
    _rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % SERIAL_BUFFER_SIZE;
    return c;
  }
}

int HardwareSerial::available(void) {    
   return (unsigned int)(SERIAL_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) % SERIAL_BUFFER_SIZE;
}

size_t HardwareSerial::write(uint8_t ch) {
	while(uart_device->FSR & UART_FSR_TX_FULL_Msk);
	uart_device->THR = ch;
}

void HardwareSerial::flush(void) {
    
}


int HardwareSerial::peek( void ) {	
	if (_rx_buffer->head == _rx_buffer->tail) {
		return -1;
	}else{
		return _rx_buffer->buffer[_rx_buffer->tail];
	}
}

