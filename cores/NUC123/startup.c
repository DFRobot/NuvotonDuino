/**
 * \file
 *
 * \brief gcc starttup file for SAMD21
 *
 * Copyright (c) 2013-2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include "NUC123.h"

/* Initialize segments */
extern uint32_t _sfixed;
extern uint32_t _efixed;
extern uint32_t _etext;
extern uint32_t _srelocate;
extern uint32_t _erelocate;
extern uint32_t _szero;
extern uint32_t _ezero;
extern uint32_t _sstack;
extern uint32_t _estack;

/** \cond DOXYGEN_SHOULD_SKIP_THIS */
int main(void);
/** \endcond */

void __libc_init_array(void);

/* Default empty handler */
void Reset_Handler(void);
void Dummy_Handler(void);
void Default_IRQHandler(void);


/* Cortex-M0 core handlers */
void NMI_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void HardFault_Handler       ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SVC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PendSV_Handler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SysTick_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));

/* Peripherals handlers */
void BOD_IRQHandler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void WDT_IRQHandler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EINT0_IRQHandler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EINT1_IRQHandler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPAB_IRQHandler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPCDF_IRQHandler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PWMA_IRQHandler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void USB_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TMR0_IRQHandler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TMR1_IRQHandler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TMR2_IRQHandler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TMR3_IRQHandler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void UART0_IRQHandler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void UART1_IRQHandler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SPI0_IRQHandler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SPI1_IRQHandler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SPI2_IRQHandler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SPI3_IRQHandler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I2C0_IRQHandler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I2C1_IRQHandler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void CAN0_IRQHandler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void CAN1_IRQHandler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SC012_IRQHandler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void USBD_IRQHandler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PS2_IRQHandler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ACMP_IRQHandler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PDMA_IRQHandler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I2S_IRQHandler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PWRWU_IRQHandler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC_IRQHandler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RTC_IRQHandler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
typedef struct _DeviceVectors
{
  /* Stack pointer */
  void* pvStack;

  /* Cortex-M handlers */
  void* pfnReset_Handler;
  void* pfnNMI_Handler;
  void* pfnHardFault_Handler;
  void* pfnReservedM12;
  void* pfnReservedM11;
  void* pfnReservedM10;
  void* pfnReservedM9;
  void* pfnReservedM8;
  void* pfnReservedM7;
  void* pfnReservedM6;
  void* pfnSVC_Handler;
  void* pfnReservedM4;
  void* pfnReservedM3;
  void* pfnPendSV_Handler;
  void* pfnSysTick_Handler;

  /* Peripheral handlers */
  void*  pfnBOD_IRQHandler;
  void*  pfnWDT_IRQHandler;
  void*  pfnEINT0_IRQHandler;
  void*  pfnEINT1_IRQHandler;
  void*  pfnGPAB_IRQHandler;
  void*  pfnGPCDF_IRQHandler;
  void*  pfnPWMA_IRQHandler;
  void*  pfnDefault_IRQHandler;
  void*  pfnTMR0_IRQHandler;
  void*  pfnTMR1_IRQHandler;
  void*  pfnTMR2_IRQHandler;
  void*  pfnTMR3_IRQHandler;
  void*  pfnUART0_IRQHandler;
  void*  pfnUART1_IRQHandler;
  void*  pfnSPI0_IRQHandler;
  void*  pfnSPI1_IRQHandler;
  void*  pfnSPI2_IRQHandler;
  void*  pfnSPI3_IRQHandler;
  void*  pfnI2C0_IRQHandler;
  void*  pfnI2C1_IRQHandler;
  void*  pfnCAN0_IRQHandler;
  void*  pfnCAN1_IRQHandler;
  void*  pfnSC012_IRQHandler;
  void*  pfnUSBD_IRQHandler;
  void*  pfnPS2_IRQHandler;
  void*  pfnACMP_IRQHandler;
  void*  pfnPDMA_IRQHandler;
  void*  pfnI2S_IRQHandler;
  void*  pfnPWRWU_IRQHandler;
  void*  pfnADC_IRQHandler;
  void*  pfnDefluat_IRQHandler;
  void*  pfnRTC_IRQHandler;
  uint32_t unused1[16];
  uint32_t unused2[64];
  uint32_t unused3[64];
} DeviceVectors;
/* Exception Table */
__attribute__ ((section(".vectors")))
const DeviceVectors exception_table = {

        /* Configure Initial Stack Pointer, using linker-generated symbols */
        (void*) (&_estack),

        (void*) Reset_Handler,
        (void*) NMI_Handler,
        (void*) HardFault_Handler,
        (void*) (0UL), /* Reserved */
        (void*) (0UL), /* Reserved */
        (void*) (0UL), /* Reserved */
        (void*) (0UL), /* Reserved */
        (void*) (0UL), /* Reserved */
        (void*) (0UL), /* Reserved */
        (void*) (0UL), /* Reserved */
        (void*) SVC_Handler,
        (void*) (0UL), /* Reserved */
        (void*) (0UL), /* Reserved */
        (void*) PendSV_Handler,
        (void*) SysTick_Handler,
//16¸ö
  (void*)  BOD_IRQHandler,
  (void*)  WDT_IRQHandler,
  (void*)  EINT0_IRQHandler,
  (void*)  EINT1_IRQHandler,
  (void*)  GPAB_IRQHandler,
  (void*)  GPCDF_IRQHandler,
  (void*)  PWMA_IRQHandler,
  (void*)  Default_IRQHandler,
  (void*)  TMR0_IRQHandler,
  (void*)  TMR1_IRQHandler,
  (void*)  TMR2_IRQHandler,
  (void*)  TMR3_IRQHandler,
  (void*)  UART0_IRQHandler,
  (void*)  UART1_IRQHandler,
  (void*)  SPI0_IRQHandler,
  (void*)  SPI1_IRQHandler,
  (void*)  SPI2_IRQHandler,
  (void*)  SPI3_IRQHandler,
  (void*)  I2C0_IRQHandler,
  (void*)  I2C1_IRQHandler,
  (void*)  CAN0_IRQHandler,
  (void*)  CAN1_IRQHandler,
  (void*)  SC012_IRQHandler,
  (void*)  USBD_IRQHandler,
  (void*)  PS2_IRQHandler,
  (void*)  ACMP_IRQHandler,
  (void*)  PDMA_IRQHandler,
  (void*)  I2S_IRQHandler,
  (void*)  PWRWU_IRQHandler,
  (void*)  ADC_IRQHandler,
  (void*)  Default_IRQHandler,
  (void*)  RTC_IRQHandler	
};

/**
 * \brief This is the code that gets called on processor reset.
 * To initialize the device, and call the main() routine.
 */
extern void SYS_Init(void);
extern void VCOM_ClassRequest(void);
void Reset_Handler(void)
{
	uint32_t *pSrc, *pDest;

	/* Initialize the relocate segment */
	pSrc = &_etext;
	pDest = &_srelocate;

	if (pSrc != pDest) {
		for (; pDest < &_erelocate;) {
			*pDest++ = *pSrc++;
		}
	}

	/* Clear the zero segment */
	for (pDest = &_szero; pDest < &_ezero;) {
		*pDest++ = 0;
	}

	/* Set the vector table base address */
	pSrc = (uint32_t *) & _sfixed;
 	//SCB->VTOR = ((uint32_t) pSrc & SCB_VTOR_TBLOFF_Msk);

	/* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();
	
    /* Init UART0 for printf */
    //UART0_Init();
	SysTick_Config(SystemCoreClock/1000);
	NVIC_SetPriority(SysTick_IRQn, 0);
	NVIC_EnableIRQ(SysTick_IRQn);
	//#if defined(__NUC123__)
	USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);
	VCOM_Init();
	USBD_Start();
	NVIC_SetPriority(USBD_IRQn, 1);
	NVIC_EnableIRQ(USBD_IRQn);
	//#endif
	/* Initialize the C library */
	__libc_init_array();

	/* Branch to main function */
	main();

	/* Infinite loop */
	while (1);
}

/**
 * \brief Default interrupt handler for unused IRQs.
 */
void Dummy_Handler(void)
{
        while (1) {
        }
}
void Default_IRQHandler(void)
{
        while (1) {
        }
}