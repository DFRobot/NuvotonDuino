

/* Copyright (c) 2011, Peter Barrett  
**  
** Permission to use, copy, modify, and/or distribute this software for  
** any purpose with or without fee is hereby granted, provided that the  
** above copyright notice and this permission notice appear in all copies.  
** 
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL  
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED  
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR  
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES  
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,  
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,  
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS  
** SOFTWARE.  
*/

#include "USBAPI.h"
#include "cdc_serial.h"
extern volatile uint32_t gu32RxSize;

//#if defined(__NUC123__)
#define CDC_ENABLED

volatile LineInfo _usbLineInfo = { 115200, 0x00, 0x00, 0x00, 0x00 ,0x00};

//extern volatile unsigned char usbRxBuf[256];
//extern volatile int usbRxHead,usbRxTail;

#define WEAK __attribute__ ((weak))

void Serial_::begin(unsigned long /* baud_count */)
{
	peek_buffer = -1;
}

void Serial_::begin(unsigned long /* baud_count */, byte /* config */)
{
	peek_buffer = -1;
}

void Serial_::end(void)
{
}

int Serial_::available(void)
{
	return USB_Available(0);
}

int Serial_::peek(void)
{
	if(usbRxTail-usbRxHead){
		return usbRxBuf[0];
	}else{
		return -1;
	}
}

int Serial_::read(void)
{
	/*if (peek_buffer >= 0) {
		int c = peek_buffer;
		peek_buffer = -1;
		return c;
	}
	return gu32RxSize;*/
	if(usbRxTail-usbRxHead){
		return usbRxBuf[usbRxHead++];
	}else{
		return -1;
	}
}

void Serial_::flush(void)
{
	//USB_Flush(0);
}

size_t Serial_::write(uint8_t c)
{
	//USB_Write(&c,1);
	//Serial1.print("open=");Serial1.println(_usbLineInfo.isOpened);
	if (_usbLineInfo.isOpened){
		digitalWrite(TX_LED,LOW);
		gi8BulkInReady = 0;
		USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), &c, 1);
		USBD_SET_PAYLOAD_LEN(EP2, 1);
		while(gi8BulkInReady == 0);
		gi8BulkInReady = 0;
		digitalWrite(TX_LED,HIGH);
		return 1;
	}
	return 1;
	//return write(&c, 1);
}

size_t Serial_::write(void *buffer, size_t size)
{
	volatile size_t current = 0,left = size,sum=0;
	/* only try to send bytes if the high-level CDC connection itself 
	 is open (not just the pipe) - the OS should set lineState when the port
	 is opened and clear lineState when the port is closed.
	 bytes sent before the user opens the connection or after
	 the connection is closed are lost - just like with a UART. */
	
	// TODO - ZE - check behavior on different OSes and test what happens if an
	// open connection isn't broken cleanly (cable is yanked out, host dies
	// or locks up, or host virtual serial port hangs)
	if (_usbLineInfo.isOpened)	{
		digitalWrite(TX_LED,LOW);
		while(left){
			size_t count = left>64?64:left;
			USB_Write(((const uint8_t*)buffer)+size-left,count);
			left -= count;
		}
		digitalWrite(TX_LED,HIGH);
		return size;
	}
	return size;
}

size_t Serial_::write(const uint8_t *buffer, size_t size)
{
	return write((void *)buffer,size);
}

// This operator is a convenient way for a sketch to check whether the
// port has actually been configured and opened by the host (as opposed
// to just being connected to the host).  It can be used, for example, in 
// setup() before printing to ensure that an application on the host is
// actually ready to receive and display the data.
// We add a short delay before returning to fix a bug observed by Federico
// where the port is configured (lineState != 0) but not quite opened.
Serial_::operator bool() {
	bool result = false;
	if (_usbLineInfo.lineState > 0) 
		result = true;
	delay(10);
	return result;
}
#if defined(__NUC123__)
Serial_ Serial;
#elif  defined(__BlunoM0__)
Serial_ USBSerial;
#endif
//#endif

