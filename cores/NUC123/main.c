
//#include <stdio.h>
#include "Arduino.h"
#include <nuc123.h>
#include "cdc_serial.h"
//#include "USBAPI.h"

#define PLLCON_SETTING      CLK_PLLCON_72MHz_HXT
#define PLL_CLOCK           72000000

volatile uint8_t init_flag=0;	


void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for IRC22M clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT, CLK_CLKDIV_HCLK(1));
	
    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
	CLK->PLLCON |= CLK_PLLCON_PD_Msk;

    /* Enable external 12MHz XTAL, internal 10KHz */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk | CLK_PWRCON_OSC10K_EN_Msk);

	/* Waiting for clock ready */
    while(!CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk | CLK_CLKSTATUS_OSC10K_STB_Msk));

    /* Enable PLL and Set PLL frequency */
	CLK_SetCoreClock(PLL_CLOCK);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_PLL_STB_Msk | CLK_CLKSTATUS_XTL12M_STB_Msk | CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to PLL, STCLK to HCLK/2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_PLL, CLK_CLKDIV_HCLK(2));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PDMA module clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMA_EN_Msk;

    /* Select UART module clock source */
    //CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    //CLK_EnableModuleClock(UART0_MODULE);
    //CLK_EnableModuleClock(TMR0_MODULE);
    //CLK_EnableModuleClock(TMR1_MODULE);	
	//CLK_EnableModuleClock(TMR2_MODULE);	
	//CLK_EnableModuleClock(TMR3_MODULE);	
    
	//CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HXT, 0);
	//CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HCLK, 0);
	//CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2_S_HIRC, 0);
    //CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3_S_LIRC, 0);

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);
    /* ADC clock source is 22.1184MHz, set divider to 7, ADC clock is 22.1184/7 MHz */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_HIRC, CLK_CLKDIV_ADC(7));

	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

	/* Enable PWM module clock */
    CLK_EnableModuleClock(PWM01_MODULE);
    CLK_EnableModuleClock(PWM23_MODULE);
	/* Select PWM module clock source */
    CLK_SetModuleClock(PWM01_MODULE, CLK_CLKSEL1_PWM01_S_HXT, 0);
    CLK_SetModuleClock(PWM23_MODULE, CLK_CLKSEL1_PWM23_S_HXT, 0);

	CLK_EnableModuleClock(USBD_MODULE);	
	CLK_SetModuleClock(USBD_MODULE, 0, CLK_CLKDIV_USB(3));
	 
	/* Enable I2C module clock */
	CLK_EnableModuleClock(I2C0_MODULE);
	/* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI1_MODULE);
    /* Reset PWMA channel0~channel3 */
    SYS_ResetModule(PWM03_RST);


	SYS->ALT_MFP &= ~SYS_ALT_MFP_PC13_CLKO;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    //SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    //SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);
	//SYS->GPA_MFP = SYS_GPA_MFP_PA12_PWM0 | SYS_GPA_MFP_PA13_PWM1 | SYS_GPA_MFP_PA14_PWM2 | SYS_GPA_MFP_PA15_PWM3;

	 //IIC
	//SYS->GPF_MFP |= (SYS_GPF_MFP_PF2_I2C0_SDA | SYS_GPF_MFP_PF3_I2C0_SCL);
    //SYS->ALT_MFP1 &= ~(SYS_ALT_MFP1_PF2_Msk | SYS_ALT_MFP1_PF3_Msk);
    //SYS->ALT_MFP1 |= (SYS_ALT_MFP1_PF2_I2C0_SDA | SYS_ALT_MFP1_PF3_I2C0_SCL);

	/* Disable the GPD0 - GPD5 digital input path to avoid the leakage current. */
    //GPIO_DISABLE_DIGITAL_PATH(PD, 0x3F);

    /* Configure the GPD0 - GPD5 ADC analog input pins */
    //SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD0_Msk | SYS_GPD_MFP_PD1_Msk | SYS_GPD_MFP_PD2_Msk | SYS_GPD_MFP_PD3_Msk | SYS_GPD_MFP_PD4_Msk | SYS_GPD_MFP_PD5_Msk) ;
    //SYS->GPD_MFP |= SYS_GPD_MFP_PD0_ADC0 | SYS_GPD_MFP_PD1_ADC1 | SYS_GPD_MFP_PD2_ADC2 | SYS_GPD_MFP_PD3_ADC3 | SYS_GPD_MFP_PD4_ADC4 | SYS_GPD_MFP_PD5_ADC5 ;
    //SYS->ALT_MFP1 &= ~(SYS_ALT_MFP1_PD0_Msk | SYS_ALT_MFP1_PD1_Msk | SYS_ALT_MFP1_PD2_Msk | SYS_ALT_MFP1_PD3_Msk | SYS_ALT_MFP1_PD4_Msk | SYS_ALT_MFP1_PD5_Msk);
    //SYS->ALT_MFP1 |= SYS_ALT_MFP1_PD0_ADC0 | SYS_ALT_MFP1_PD1_ADC1 | SYS_ALT_MFP1_PD2_ADC2 | SYS_ALT_MFP1_PD3_ADC3 | SYS_ALT_MFP1_PD4_ADC4 | SYS_ALT_MFP1_PD5_ADC5;

	
	/* Setup SPI0 multi-function pins */
    //SYS->GPC_MFP = SYS_GPC_MFP_PC0_SPI0_SS0 | SYS_GPC_MFP_PC1_SPI0_CLK | SYS_GPC_MFP_PC2_SPI0_MISO0 | SYS_GPC_MFP_PC3_SPI0_MOSI0;
    //SYS->ALT_MFP = SYS_ALT_MFP_PC0_SPI0_SS0 | SYS_ALT_MFP_PC1_SPI0_CLK | SYS_ALT_MFP_PC2_SPI0_MISO0 | SYS_ALT_MFP_PC3_SPI0_MOSI0;
}
char printbuf[256];
#define uart_printf(...){\
  sprintf(printbuf,__VA_ARGS__);\
  UART_WriteString(printbuf);\
}
/*
void VCOM_TransferData(void)
{
	int len;
	if(gi8BulkOutReady){
		len = USB_Read(msgBuffer);
		USB_Write(msgBuffer,len);
	}
}*/
size_t mywrite(void *buffer,size_t size)
{
	size_t left = size;
	int i32Len;
	if (_usbLineInfo.lineState > 0) {
		while(left){
			size_t count = left>64?64:left;
			USB_Write(((const uint8_t*)buffer)+size-left,count);
			__set_PRIMASK(1);
			left -= count;
            __set_PRIMASK(0);
		}
		i32Len = USBD_GET_PAYLOAD_LEN(EP2);
		if(i32Len == EP2_MAX_PKT_SIZE)
			USBD_SET_PAYLOAD_LEN(EP2, 0);
		return size;
	}
	return 0;
}

int main(void)
{
	int i;
	hook();
	for(i=0;i<31;i++)
		pinMode(i,INPUT);
	setup();
	//uart_printf("111111\r\n");
	//PB4=0;
	//delay(1000);
	//uart_printf("22222\r\n");
	//char buf1[100]={"abcdefghijabcdefghijabcdefghijabcdefghijabcdefghijabcdefghijabcdefghijabcdefghijabcdefghijabcdefg\r\n"};
	//char buf2[10]={"begin\r\n"};	
	while(1)
	{
		//mywrite(buf1,100);
		//mywrite(buf2,8);
		//delay(1000);
		//unsigned char buf[100];
		//while(gi8BulkOutReady == 0);
		//USB_Read(buf);
		//USB_Write(buf,5);
		//Serial.write("wanghui",7);
		//uart_printf(printbuf,"33333\r\n");
		//Serial.write("nihao",5);
		//delay(1000);
		loop();
		//#if defined(__BlunoM0__)	
	    //UDC_MainProcess();					//en: Execute MassStorage process
		//#endif
	}
	return 0;
}

