#include <Arduino.h>
#include "NUC123.h"
#include "pins_arduino.h"

#if defined(__NUC123__)
const GPIOPinDescription GPIO_Desc[]=
{
	{PC,BIT4  ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC4_Msk, (uint32_t)&SYS->ALT_MFP  , SYS_ALT_MFP_PC4_Msk}},  //0/RX
	{PC,BIT5  ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC5_Msk, (uint32_t)&SYS->ALT_MFP  , SYS_ALT_MFP_PC5_Msk}},  //1/TX
	{PF,BIT2  ,{(uint32_t)&SYS->GPF_MFP, SYS_GPF_MFP_PF2_Msk, (uint32_t)&SYS->ALT_MFP1 , SYS_ALT_MFP1_PF2_Msk}},  //2/ SDA
	{PF,BIT3  ,{(uint32_t)&SYS->GPF_MFP, SYS_GPF_MFP_PF3_Msk, (uint32_t)&SYS->ALT_MFP1 , SYS_ALT_MFP1_PF3_Msk}},  //3/ SCL  //PWMxxxxxx
	
	{PC,BIT0  ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC0_Msk, (uint32_t)&SYS->ALT_MFP  , SYS_ALT_MFP_PC0_Msk}},  //4/	I2S_LRCLK
	{PA,BIT12 ,{(uint32_t)&SYS->GPA_MFP, SYS_GPA_MFP_PA12_Msk, NULL,NULL}},  //5  //PWM0
	{PA,BIT13 ,{(uint32_t)&SYS->GPA_MFP, SYS_GPA_MFP_PA13_Msk, NULL,NULL}},  //6  //PWM1
	{PC,BIT2  ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC2_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PC2_Msk}},  //7  //I2S_DI

	{PC,BIT3  ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC3_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PC3_Msk}},  //8  //I2S_DO
	{PA,BIT14 ,{(uint32_t)&SYS->GPA_MFP, SYS_GPA_MFP_PA14_Msk, NULL,NULL}},  //9    //PWM2
	{PA,BIT15 ,{(uint32_t)&SYS->GPA_MFP, SYS_GPA_MFP_PA15_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PA15_Msk}},  //10   //PWM3
	{PB,BIT8  ,{(uint32_t)&SYS->GPB_MFP, SYS_GPB_MFP_PB8_Msk, NULL,0}},  //11   //PWM4 Soft

	{PC,BIT1  ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC1_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PC1_Msk}},  //12   //I2S_BCLK
	{PB,BIT14 ,{(uint32_t)&SYS->GPB_MFP, SYS_GPB_MFP_PB14_Msk, NULL,0}},  //13   //PWM5 Soft
	{PA,BIT10 ,{(uint32_t)&SYS->GPA_MFP, SYS_GPA_MFP_PA10_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PA10_Msk}},  //14   //SPI1_MISO
	{PA,BIT11 ,{(uint32_t)&SYS->GPA_MFP, SYS_GPA_MFP_PA11_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PA11_Msk}},  //15   //SPI1_SCK

	{PC,BIT11 ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC11_Msk, NULL,0}},  //16   //SPI1_MOSI

	{PB,BIT6 , {(uint32_t)&SYS->GPB_MFP, SYS_GPB_MFP_PB6_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PB6_Msk}},  //17 //RXLED

	{PD,BIT0 , {(uint32_t)&SYS->GPD_MFP, SYS_GPD_MFP_PD0_Msk, (uint32_t)&SYS->ALT_MFP1 ,SYS_ALT_MFP1_PD0_Msk}},  //18 A0
	{PD,BIT1 , {(uint32_t)&SYS->GPD_MFP, SYS_GPD_MFP_PD1_Msk, (uint32_t)&SYS->ALT_MFP1 ,SYS_ALT_MFP1_PD1_Msk}},  //19 A1
	{PD,BIT2 , {(uint32_t)&SYS->GPD_MFP, SYS_GPD_MFP_PD2_Msk, (uint32_t)&SYS->ALT_MFP1 ,SYS_ALT_MFP1_PD2_Msk}},  //20 A2
	{PD,BIT3 , {(uint32_t)&SYS->GPD_MFP, SYS_GPD_MFP_PD3_Msk, (uint32_t)&SYS->ALT_MFP1 ,SYS_ALT_MFP1_PD3_Msk}},  //21 A3
	{PD,BIT4 , {(uint32_t)&SYS->GPD_MFP, SYS_GPD_MFP_PD4_Msk, (uint32_t)&SYS->ALT_MFP1 ,SYS_ALT_MFP1_PD4_Msk}},  //22 A4
	{PD,BIT5 , {(uint32_t)&SYS->GPD_MFP, SYS_GPD_MFP_PD5_Msk, (uint32_t)&SYS->ALT_MFP1 ,SYS_ALT_MFP1_PD5_Msk}},  //23 A5
	
	{PB,BIT4 , {(uint32_t)&SYS->GPB_MFP, SYS_GPB_MFP_PB4_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PB4_Msk}},  //24 //RXD1
	{PB,BIT5 , {(uint32_t)&SYS->GPB_MFP, SYS_GPB_MFP_PB5_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PB5_Msk}},  //25 //TXD1
	{PC,BIT10 ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC10_Msk, NULL,NULL}},  //26 //
	{PC,BIT9 , {(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC9_Msk, NULL,NULL}},  //27 //
	{PC,BIT13 ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC13_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PC13_Msk}},  //28 //
	{PC,BIT12 ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC12_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PC12_Msk}},  //29 //
	{PB,BIT9 , {(uint32_t)&SYS->GPB_MFP, SYS_GPB_MFP_PB9_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PB9_Msk}},  //30 //
	{PB,BIT10 ,{(uint32_t)&SYS->GPB_MFP, SYS_GPB_MFP_PB10_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PB10_Msk}},  //31 //
	{PB,BIT7 , {(uint32_t)&SYS->GPB_MFP, SYS_GPB_MFP_PB7_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PB7_Msk}},  //32 //TXLED

};			
#elif defined(__BlunoM0__)
const GPIOPinDescription GPIO_Desc[]=
{
	{PC,BIT4  ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC4_Msk, (uint32_t)&SYS->ALT_MFP  , SYS_ALT_MFP_PC4_Msk}},  //0/RX
	{PC,BIT5  ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC5_Msk, (uint32_t)&SYS->ALT_MFP  , SYS_ALT_MFP_PC5_Msk}},  //1/TX
	{PF,BIT2  ,{(uint32_t)&SYS->GPF_MFP, SYS_GPF_MFP_PF2_Msk, (uint32_t)&SYS->ALT_MFP1 , SYS_ALT_MFP1_PF2_Msk}},  //2/ SDA
	{PF,BIT3  ,{(uint32_t)&SYS->GPF_MFP, SYS_GPF_MFP_PF3_Msk, (uint32_t)&SYS->ALT_MFP1 , SYS_ALT_MFP1_PF3_Msk}},  //3/ SCL  //PWMxxxxxx
	
	{PC,BIT0  ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC0_Msk, (uint32_t)&SYS->ALT_MFP  , SYS_ALT_MFP_PC0_Msk}},  //4/	I2S_LRCLK
	{PA,BIT12 ,{(uint32_t)&SYS->GPA_MFP, SYS_GPA_MFP_PA12_Msk, NULL,NULL}},  //5  //PWM0
	{PA,BIT13 ,{(uint32_t)&SYS->GPA_MFP, SYS_GPA_MFP_PA13_Msk, NULL,NULL}},  //6  //PWM1
	{PC,BIT2  ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC2_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PC2_Msk}},  //7  //I2S_DI

	{PC,BIT3  ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC3_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PC3_Msk}},  //8  //I2S_DO
	{PA,BIT14 ,{(uint32_t)&SYS->GPA_MFP, SYS_GPA_MFP_PA14_Msk, NULL,NULL}},  //9    //PWM2
	{PA,BIT15 ,{(uint32_t)&SYS->GPA_MFP, SYS_GPA_MFP_PA15_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PA15_Msk}},  //10   //PWM3
	{PB,BIT8  ,{(uint32_t)&SYS->GPB_MFP, SYS_GPB_MFP_PB8_Msk, NULL,0}},  //11   //PWM4 Soft

	{PC,BIT1  ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC1_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PC1_Msk}},  //12   //I2S_BCLK
	{PB,BIT14 ,{(uint32_t)&SYS->GPB_MFP, SYS_GPB_MFP_PB14_Msk, NULL,0}},  //13   //PWM4 Soft
	{PA,BIT10 ,{(uint32_t)&SYS->GPA_MFP, SYS_GPA_MFP_PA10_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PA10_Msk}},  //14   //SPI1_MISO
	{PA,BIT11 ,{(uint32_t)&SYS->GPA_MFP, SYS_GPA_MFP_PA11_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PA11_Msk}},  //15   //SPI1_SCK

	{PC,BIT11 ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC11_Msk, NULL,0}},  //16   //SPI1_MOSI

	{PB,BIT6 , {(uint32_t)&SYS->GPB_MFP, SYS_GPB_MFP_PB6_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PB6_Msk}},  //17 //RXLED

	{PD,BIT0 , {(uint32_t)&SYS->GPD_MFP, SYS_GPD_MFP_PD0_Msk, (uint32_t)&SYS->ALT_MFP1 ,SYS_ALT_MFP1_PD0_Msk}},  //18 A0
	{PD,BIT1 , {(uint32_t)&SYS->GPD_MFP, SYS_GPD_MFP_PD1_Msk, (uint32_t)&SYS->ALT_MFP1 ,SYS_ALT_MFP1_PD1_Msk}},  //19 A1
	{PD,BIT2 , {(uint32_t)&SYS->GPD_MFP, SYS_GPD_MFP_PD2_Msk, (uint32_t)&SYS->ALT_MFP1 ,SYS_ALT_MFP1_PD2_Msk}},  //20 A2
	{PD,BIT3 , {(uint32_t)&SYS->GPD_MFP, SYS_GPD_MFP_PD3_Msk, (uint32_t)&SYS->ALT_MFP1 ,SYS_ALT_MFP1_PD3_Msk}},  //21 A3
	{PD,BIT4 , {(uint32_t)&SYS->GPD_MFP, SYS_GPD_MFP_PD4_Msk, (uint32_t)&SYS->ALT_MFP1 ,SYS_ALT_MFP1_PD4_Msk}},  //22 A4
	{PD,BIT5 , {(uint32_t)&SYS->GPD_MFP, SYS_GPD_MFP_PD5_Msk, (uint32_t)&SYS->ALT_MFP1 ,SYS_ALT_MFP1_PD5_Msk}},  //23 A5
	
	{PB,BIT4 , {(uint32_t)&SYS->GPB_MFP, SYS_GPB_MFP_PB4_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PB4_Msk}},  //24 //RXD1
	{PB,BIT5 , {(uint32_t)&SYS->GPB_MFP, SYS_GPB_MFP_PB5_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PB5_Msk}},  //25 //TXD1
	{PC,BIT10 ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC10_Msk, NULL,NULL}},  //26 //
	{PC,BIT9 , {(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC9_Msk, NULL,NULL}},  //27 //
	{PC,BIT13 ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC13_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PC13_Msk}},  //28 //
	{PC,BIT12 ,{(uint32_t)&SYS->GPC_MFP, SYS_GPC_MFP_PC12_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PC12_Msk}},  //29 //
	{PB,BIT9 , {(uint32_t)&SYS->GPB_MFP, SYS_GPB_MFP_PB9_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PB9_Msk}},  //30 //
	{PB,BIT10 ,{(uint32_t)&SYS->GPB_MFP, SYS_GPB_MFP_PB10_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PB10_Msk}},  //31 //
	{PB,BIT7 , {(uint32_t)&SYS->GPB_MFP, SYS_GPB_MFP_PB7_Msk, (uint32_t)&SYS->ALT_MFP  ,SYS_ALT_MFP_PB7_Msk}},  //32 //TXLED

};
#endif

const ADCPinDescription ADC_Desc[]={
	{ADC,ADC_MODULE,0,{1UL<<0, 1UL<<16}},   //0//
	{ADC,ADC_MODULE,1,{1UL<<1, 1UL<<17}},	//1//
	{ADC,ADC_MODULE,2,{1UL<<2, 1UL<<18}},	//2//
	{ADC,ADC_MODULE,3,{1UL<<3, 1UL<<19}},	//3//
	{ADC,ADC_MODULE,4,{1UL<<4, 1UL<<20}},	//4//
	{ADC,ADC_MODULE,5,{1UL<<5, 1UL<<21}},	//5//
};

const PWMPinDescription PWM_Desc[]={
	{PWMA,PWM01_MODULE,PWMA_IRQn,0,500, 5,{0,0}},    //0//
	{PWMA,PWM01_MODULE,PWMA_IRQn,1,500, 6,{0,0}},    //1//
	{PWMA,PWM23_MODULE,PWMA_IRQn,2,500, 9,{0,0}},    //2//
	{PWMA,PWM23_MODULE,PWMA_IRQn,3,500,10,{0,0}}     //3//
};
const u32  digital_pin_to_port[] = {
	PC, /* 0 */
	PC,
	PF,
	PF,
	PC,
	PA,
	PA,
	PC,
	PC, /* 8 */
	PA,
	PA,
	PB,
	PC,
	PB,
	PA, /* 14 */
	PA,
	PC,
	PC,
	PC,
	PC,
};

const u32 digital_pin_to_bit_mask[] = {
	(1L<<4), 
	(1L<<5),
	(1L<<2),
	(1L<<3),
	
	(1L<<0),
	(1L<<12),
	(1L<<13),
	(1L<<2),
	
	(1L<<3),
	(1L<<14),
	(1L<<15),
	(1L<<4),
	
	(1L<<1),
	(1L<<14),
	(1L<<10),
	(1L<<11),
	
	(1L<<11),
	(1L<<3),
	(1L<<4),
	(1L<<5),
};

volatile u32* digital_pin_to_input[] = {
	(volatile u32*) ((GPIO_PIN_DATA_BASE+(0x40*(2))) + ((4)<<2)), /* 0 */
	(volatile u32*) ((GPIO_PIN_DATA_BASE+(0x40*(2))) + ((5)<<2)),
	(volatile u32*) ((GPIO_PIN_DATA_BASE+(0x40*(5))) + ((2)<<2)),
	(volatile u32*) ((GPIO_PIN_DATA_BASE+(0x40*(5))) + ((3)<<2)),

	(volatile u32*) ((GPIO_PIN_DATA_BASE+(0x40*(2))) + ((0)<<2)),
	(volatile u32*) ((GPIO_PIN_DATA_BASE+(0x40*(0))) + ((12)<<2)),
	(volatile u32*) ((GPIO_PIN_DATA_BASE+(0x40*(0))) + ((13)<<2)),
	(volatile u32*) ((GPIO_PIN_DATA_BASE+(0x40*(2))) + ((2)<<2)),

	(volatile u32*) ((GPIO_PIN_DATA_BASE+(0x40*(2))) + ((3)<<2)),
	(volatile u32*) ((GPIO_PIN_DATA_BASE+(0x40*(0))) + ((14)<<2)),
	(volatile u32*) ((GPIO_PIN_DATA_BASE+(0x40*(0))) + ((15)<<2)),
	(volatile u32*) ((GPIO_PIN_DATA_BASE+(0x40*(1))) + ((4)<<2)),

	(volatile u32*) ((GPIO_PIN_DATA_BASE+(0x40*(2))) + ((1)<<2)),
	(volatile u32*) ((GPIO_PIN_DATA_BASE+(0x40*(1))) + ((14)<<2)),
	(volatile u32*) ((GPIO_PIN_DATA_BASE+(0x40*(0))) + ((10)<<2)),
	(volatile u32*) ((GPIO_PIN_DATA_BASE+(0x40*(0))) + ((11)<<2)),
};

