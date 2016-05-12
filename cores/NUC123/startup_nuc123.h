#ifndef __STARTUP_NUC123_H
#define __STARTUP_NUC123_H

#ifdef __cplusplus
extern "C" {
#endif

#define DYNAMIC_HANDLERS

/* Cortex-M0 core handlers */
void NMI_Handler        ( void );
void HardFault_Handler  ( void );
void SVC_Handler        ( void );
void PendSV_Handler     ( void );
void SysTick_Handler    ( void );

/* peripheral handlers */
void BOD_IRQHandler         ( void );
void WDT_IRQHandler         ( void );
void EINT0_IRQHandler       ( void );
void EINT1_IRQHandler       ( void );
void GPAB_IRQHandler        ( void );
void GPCDF_IRQHandler       ( void );
void PWMA_IRQHandler        ( void );
void Default_IRQHandler     ( void );
void TMR0_IRQHandler        ( void );
void TMR1_IRQHandler        ( void );
void TMR2_IRQHandler        ( void );
void TMR3_IRQHandler        ( void );
void UART0_IRQHandler       ( void );
void UART1_IRQHandler       ( void );
void SPI0_IRQHandler        ( void );
void SPI1_IRQHandler        ( void );
void SPI2_IRQHandler        ( void );
void SPI3_IRQHandler        ( void );
void I2C0_IRQHandler        ( void );
void I2C1_IRQHandler        ( void );
void CAN0_IRQHandler        ( void );
void CAN1_IRQHandler        ( void );
void SC012_IRQHandler       ( void );
void USBD_IRQHandler        ( void );
void PS2_IRQHandler         ( void );
void ACMP_IRQHandler        ( void );
void PDMA_IRQHandler        ( void );
void I2S_IRQHandler         ( void );
void PWRWU_IRQHandler       ( void );
void ADC_IRQHandler         ( void );
void Default_IRQHandler     ( void );
void RTC_IRQHandler         ( void );

#ifdef DYNAMIC_HANDLERS

typedef void (*dynamic_handler_t)(void);
extern dynamic_handler_t *dynamic_handlers/*[32]*/;

#endif

#ifdef __cplusplus
}
#endif

#endif