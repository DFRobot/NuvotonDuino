/******************************************************************************
 * @file     cdc_serial.c
 * @brief    NUC123 series USBD VCOM sample file
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NUC123.h"
#include "usbd.h"
#include "cdc_serial.h"
#include "pins_arduino.h"

//#if defined(__NUC123__)
/*--------------------------------------------------------------------------*/
void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();

//------------------------------------------------------------------
    if(u32IntSts & USBD_INTSTS_FLDET)
    {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if(USBD_IS_ATTACHED())
        {
            /* USB Plug In */
            USBD_ENABLE_USB();
        }
        else
        {
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
    }

//------------------------------------------------------------------
    if(u32IntSts & USBD_INTSTS_BUS)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if(u32State & USBD_STATE_USBRST)
        {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
        }
        if(u32State & USBD_STATE_SUSPEND)
        {
        	_usbLineInfo.lineState = 0;
            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }
        if(u32State & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
        }
    }

//------------------------------------------------------------------
    if(u32IntSts & USBD_INTSTS_USB)
    {
        extern uint8_t g_usbd_SetupPacket[];

        // USB event
        if(u32IntSts & USBD_INTSTS_SETUP)
        {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);

            USBD_ProcessSetupPacket();
        }

        // EP events
        if(u32IntSts & USBD_INTSTS_EP0)
        {

            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);

            // control IN
            USBD_CtrlIn();
        }

        if(u32IntSts & USBD_INTSTS_EP1)
        {

            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);

            // control OUT
            USBD_CtrlOut();

            // In ACK of SET_LINE_CODE
            if(g_usbd_SetupPacket[1] == SET_LINE_CODE)
            {
                if(g_usbd_SetupPacket[4] == 0)  /* VCOM-1 */
                    VCOM_LineCoding(0); /* Apply UART settings */
            }

        }

        if(u32IntSts & USBD_INTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
            // Bulk IN
            EP2_Handler();
        }

        if(u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
            // Bulk Out
            EP3_Handler();
        }

        if(u32IntSts & USBD_INTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
        }

        if(u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
        }

        if(u32IntSts & USBD_INTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
        }

        if(u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
        }
    }
    /* clear unknown event */
    USBD_CLR_INT_FLAG(u32IntSts);
}

void EP2_Handler(void)
{
    gu32TxSize = 0;
	gi8BulkInReady = 1;
}


void EP3_Handler(void)
{

	testFunc();
	return;
	
    /* Bulk OUT */
	unsigned int  size;
	unsigned char buf[64];
	unsigned char *pBuf;
	int len;
	digitalWrite(TX_LED,LOW);
    len = USBD_GET_PAYLOAD_LEN(EP3);
    gpu8RxBuf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));
	pBuf = (unsigned char *)gpu8RxBuf;
    /* Set a flag to indicate bulk out ready */
    gi8BulkOutReady = 1;
	//memcpy((void *)buf,(const void*)pBuf,(unsigned int)len);
	
	if(len){
	//	Serial.print("len=");Serial.println(len);
	}
	if(usbRxHead){
		int i;
		for(i=0;i<usbRxTail-usbRxHead;i++){
			usbRxBuf[i] = usbRxBuf[usbRxHead+i];
		}
		usbRxTail -= usbRxHead;
		usbRxHead = 0;
	}else if(usbRxTail == 256){
			return;
	}
	if(usbRxTail+len>256){
		//memcpy((void *)&usbRxBuf[usbRxTail+1],pBuf,256-usbRxTail);
		usbRxTail = 256;
	}else{
		memcpy((void *)&usbRxBuf[usbRxTail],pBuf,len);
		usbRxTail += len;
		if(len){
		//	Serial.print("usbRxTail=");Serial.println(usbRxTail);
		//	Serial.print("usbRxHead=");Serial.println(usbRxHead);
		}
	}
	USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
	digitalWrite(TX_LED,HIGH);
}




/*--------------------------------------------------------------------------*/
/**
  * @brief  USBD Endpoint Config.
  * @param  None.
  * @retval None.
  */
void VCOM_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer for setup packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = SETUP_BUF_BASE;

    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);

    /*****************************************************/
    /* EP2 ==> Bulk IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM);
    /* Buffer offset for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* EP3 ==> Bulk Out endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM);
    /* Buffer offset for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

    /* EP4 ==> Interrupt IN endpoint, address 3 */
    USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    /* Buffer offset for EP4 ->  */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);
}


void VCOM_ClassRequest(void)
{
    uint8_t buf[8];

    USBD_GetSetupPacket(buf);

    if(buf[0] & 0x80)    /* request data transfer direction */
    {
        // Device to host
        switch(buf[1])
        {
            case GET_LINE_CODE:
            {
                if(buf[4] == 0)    /* VCOM-1 */
                {
                    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&gLineCoding, 7);
                }
                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 7);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }
            default:
            {
                /* Setup error, stall the device */
                USBD_SetStall(EP0);
                USBD_SetStall(EP1);
                break;
            }
        }
    }else{
        // Host to device
        switch(buf[1]){
            case SET_CONTROL_LINE_STATE:{
                if(buf[4] == 0){    /* VCOM-1 */
                    gCtrlSignal = buf[3];
                    gCtrlSignal = (gCtrlSignal << 8) | buf[2];
                    //printf("RTS=%d  DTR=%d\n", (gCtrlSignal0 >> 1) & 1, gCtrlSignal0 & 1);
                }
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
				if((gCtrlSignal == 1) || (gCtrlSignal == 3)){
					gi8BulkInReady = 1;
					_usbLineInfo.isOpened = 1;
					//PB14 = 0;
				}
				if((gCtrlSignal == 0) || gCtrlSignal == 2){
					//PB14 = 1;
					gi8BulkInReady = 1;
					if(gLineCoding.u32DTERate == 1200){
						*(unsigned int *)0x20004000 = 0x11223344;
						{
							SYS_UnlockReg();
							//NVIC_DisableIRQ(USBD_IRQn);
							SYS_ResetCPU();
							//WDT_Open(uint32_t u32TimeoutInterval,uint32_t u32ResetDelay,uint32_t u32EnableReset,uint32_t u32EnableWakeup)
							return;
						}
					}
					_usbLineInfo.isOpened = 0;
					_usbLineInfo.lineState = 0;
				}
                break;
            }
            case SET_LINE_CODE:{
                if(buf[4] == 0)  /* VCOM-1 */
                    USBD_PrepareCtrlOut((uint8_t *)&gLineCoding, 7);
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
				if(gLineCoding.u32DTERate == 1200){
					*(unsigned int *)0x20004000 = 0x11223344;
					{
						SYS_UnlockReg();
						//NVIC_DisableIRQ(USBD_IRQn);
						SYS_ResetCPU();
						//WDT_Open(uint32_t u32TimeoutInterval,uint32_t u32ResetDelay,uint32_t u32EnableReset,uint32_t u32EnableWakeup)
						return;
					}
				}

                break;
            }
            default:{
                // Stall
                /* Setup error, stall the device */
                USBD_SetStall(EP0);
                USBD_SetStall(EP1);
                break;
            }
        }
    }
}

void VCOM_LineCoding(uint8_t port)
{
    uint32_t u32Baudrate;
	int loop;
	_usbLineInfo.lineState = 1;
	
    if(port == 0)
    {
        u32Baudrate = gLineCoding.u32DTERate;

	}
}
//#endif

