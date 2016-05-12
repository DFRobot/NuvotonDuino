/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Arduino.h"


/** Tick Counter united by ms */
static volatile uint32_t _dwTickCount=0 ;




void		   HardFault_Handler(void){while(1);}
void		   PendSV_Handler(void){while(1);}
void		   NMI_Handler(void){while(1);}
void		   PS2_IRQHandler(void){while(1);}
void		   CAN1_IRQHandler(void){while(1);}
void		   SPI1_IRQHandler(void){while(1);}

void		   ACMP_IRQHandler(void){while(1);}
void		   RTC_IRQHandler(void){while(1);}
void		   PWRWU_IRQHandler(void){while(1);}
void		   USB_Handler(void){while(1);}

void		   EINT1_IRQHandler(void){while(1);}
void		   PWMA_IRQHandler(void){while(1);}

void		   BOD_IRQHandler(void){while(1);}
void		   SPI2_IRQHandler(void){while(1);}
void		   WDT_IRQHandler(void){while(1);}
void		   SVC_Handler(void){while(1);}
void		   SC012_IRQHandler(void){while(1);}
void		   EINT0_IRQHandler(void){while(1);}
void		   CAN0_IRQHandler(void){while(1);}

void		   SPI0_IRQHandler(void){while(1);}
void		   SPI3_IRQHandler(void){while(1);}


/**
 * \brief SysTick_Handler.
 */
void SysTick_Handler(void)
{
	/* Increment counter necessary in delay(). */
	_dwTickCount++;
}

/**
 *  \brief Get current Tick Count, in ms.
 */
uint32_t GetTickCount( void )
{
    return _dwTickCount ;
}


uint32_t millis( void )
{
// todo: ensure no interrupts
	return GetTickCount() ;
}

uint32_t micros( void )
{
    uint32_t ticks ;
    uint32_t count ;

    SysTick->CTRL;
    do {
        ticks = SysTick->VAL;
        count = GetTickCount();
    } while (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);

    return count * 1000 + (SysTick->LOAD + 1 - ticks) / (SystemCoreClock/1000000) ;  
}

void delay( uint32_t ms )
{
    uint32_t end = GetTickCount() + ms;
    while (GetTickCount() < end);
}
#if defined(__M451__)
	#define INSTRUCTION_DELAY 8
#elif defined(__NUC240__)
	#define INSTRUCTION_DELAY 20
#elif defined(__NANO100__) | defined(__NANO1X2__)
	#define INSTRUCTION_DELAY 16
#elif defined(__NUC123__)
	#define INSTRUCTION_DELAY 20
#elif defined(__BlunoM0__)
	#define INSTRUCTION_DELAY 20
#endif

void delayMicroseconds( uint32_t us )
{
	  /* The error of the instruction delay mesurement is 3 us */	  
	  if(us>INSTRUCTION_DELAY)
	  {	  	
	  	us-=INSTRUCTION_DELAY;
	  }
	  else
	  	return;	 		    
    uint32_t start = micros();
    while ((micros() - start) < us);         
}

