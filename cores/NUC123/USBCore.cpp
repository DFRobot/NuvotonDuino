#include "types.h"
#include "USBAPI.h"
#include "cdc_serial.h"
#include "pins_arduino.h"

class LockEP
{
public:
	LockEP()
	{
		NVIC_DisableIRQ(USBD_IRQn);
	}
	~LockEP()
	{
		NVIC_EnableIRQ(USBD_IRQn);
	}
};

u8 USB_Available(u8 ep)
{
	return 	usbRxTail - usbRxHead;
	unsigned char buf[64];
	int len;
	LockEP lock();
	len = USB_Read(buf);
	/*if(len){
		Serial.print("len=");Serial.println(len);
	}*/
	//if(usbRxTail+len>256){
		if(usbRxHead){
			for(int i=0;i<usbRxTail-usbRxHead;i++){
				usbRxBuf[i] = usbRxBuf[usbRxHead+i];
			}
			usbRxTail -= usbRxHead;
			usbRxHead = 0;
		}else if(usbRxTail == 256){
			return 256;
		}
		if(usbRxTail+len>256){
			memcpy((void *)&usbRxBuf[usbRxTail+1],buf,256-usbRxTail);
			usbRxTail = 256;
		}else{
			memcpy((void *)&usbRxBuf[usbRxTail],buf,len);
			usbRxTail += len;
			/*if(len){
				Serial.print("usbRxTail=");Serial.println(usbRxTail);
				Serial.print("usbRxHead=");Serial.println(usbRxHead);
			}*/
		}
	//}Serial.print("1234");
	return usbRxTail - usbRxHead;
}

void testFunc(void)
{
    /* Bulk OUT */
	unsigned int  size;
	unsigned char buf[64];
	unsigned char *pBuf;
	int len;
	digitalWrite(RX_LED,LOW);
    len = USBD_GET_PAYLOAD_LEN(EP3);
    gpu8RxBuf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));
	pBuf = (unsigned char *)gpu8RxBuf;
	memcpy(buf,pBuf,len);
	USBD_SET_PAYLOAD_LEN(EP3, 64);
	//return;
    /* Set a flag to indicate builk out ready */
    gi8BulkOutReady = 1;

	if(usbRxHead){
		int i;
		for(i=0;i<usbRxTail-usbRxHead;i++){
			usbRxBuf[i] = usbRxBuf[usbRxHead+i];
		}
		usbRxTail -= usbRxHead;
		usbRxHead = 0;
	}else if(usbRxTail == 255){
			return;
	}
	if(usbRxTail+len>255){
		memcpy((void *)&usbRxBuf[usbRxTail+1],buf,255-usbRxTail);
		usbRxTail = 255;
	}else{
		if(usbRxTail == 0)
			memcpy((void *)&usbRxBuf[usbRxTail],buf,len);
		else
			memcpy((void *)&usbRxBuf[usbRxTail+1],buf,len);
		usbRxTail += len;
	}
	digitalWrite(RX_LED,HIGH);		
}

