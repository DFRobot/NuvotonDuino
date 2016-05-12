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
 * @file HardwareSerial.h
 * @brief Wirish serial port interface.
 */

#ifndef _HARDWARESERIAL_H_
#define _HARDWARESERIAL_H_

#include "Arduino.h"

#include <inttypes.h>

#include "Stream.h"
#include "USBAPI.h"


#define SERIAL_BUFFER_SIZE 16

struct ring_buffer
{
  unsigned char buffer[SERIAL_BUFFER_SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;
};

/*
 * IMPORTANT:
 *
 * This class documented "by hand" (i.e., not using Doxygen) in the
 * leaflabs-docs/ repository.
 *
 * If you alter the public HardwareSerial interface, you MUST update
 * the documentation accordingly.
 */
class HardwareSerial : public Stream {
public:
    HardwareSerial(UART_T *uart_device,
                   ring_buffer *rx_buffer);

    /* Set up/tear down */
    void begin(uint32_t baud);
    void end(void);

    /* I/O */
    int available( void ) ;
    int peek( void ) ;
    int read( void ) ;
    void flush( void ) ;
    virtual size_t write( uint8_t ch ) ;
	//inline size_t write(unsigned long n) { return write((uint8_t)n); }
    //inline size_t write(long n) { return write((uint8_t)n); }
    //inline size_t write(unsigned int n) { return write((uint8_t)n); }
    //inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool() { return true; }; // UART always active
    
    ring_buffer *_rx_buffer;
private:
    UART_T *uart_device;
    uint32_t u32Idx;
    uint32_t u32ClkSrc;
    uint32_t u32ClkDiv;  
    IRQn_Type u32IrqId;
};
extern HardwareSerial Serial2;
#if defined(__NUC123__)
#elif defined(__BlunoM0__)
extern HardwareSerial &Serial;
#endif
extern HardwareSerial Serial1;
extern void serialEventRun(void) __attribute__((weak));

#endif

