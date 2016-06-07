#include <NUC123.h>
#include <inttypes.h>
#include "interrupt.h"
#include "USBAPI.h"

static volatile voidFuncPtr intFunc[EXTERNAL_NUM_INTERRUPTS];
#define GPIO_INT_RISING         0x00010000UL /*!< Interrupt enable by Input Rising Edge */
#define GPIO_INT_FALLING        0x00000001UL /*!< Interrupt enable by Input Falling Edge */
#define GPIO_INT_BOTH_EDGE      0x00010001UL /*!< Interrupt enable by both Rising Edge and Falling Edge */
#define GPIO_INT_HIGH           0x01010000UL /*!< Interrupt enable by Level-High */
#define GPIO_INT_LOW            0x01000001UL /*!< Interrupt enable by Level-Level */

static const unsigned int modeArray[5]={GPIO_INT_LOW, GPIO_INT_HIGH, GPIO_INT_BOTH_EDGE, GPIO_INT_FALLING, GPIO_INT_RISING};

void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), int mode) {
  if(interruptNum < EXTERNAL_NUM_INTERRUPTS) {
    intFunc[interruptNum] = userFunc;
    
    // Configure the interrupt mode (trigger on low input, any change, rising
    // edge, or falling edge).  The mode constants were chosen to correspond
    // to the configuration bits in the hardware register, so we simply shift
    // the mode into place.
      
    // Enable the interrupt.
    //D0-PB0-INT2  D1-PB1-INT3  D2-PF2-INT1  D3-PF3-INT0
      
    switch (interruptNum) {
    case 0://PF3
		GPIO_SetMode(PF, BIT3, GPIO_PMD_QUASI);
		GPIO_EnableInt(PF, 3, modeArray[mode]);
		GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, GPIO_DBCLKSEL_32);
		GPIO_ENABLE_DEBOUNCE(PF, BIT3);
		NVIC_SetPriority(GPCDF_IRQn, 2);
		NVIC_EnableIRQ(GPCDF_IRQn);
	break;
    case 1://PF2
	   GPIO_SetMode(PF, BIT2, GPIO_PMD_QUASI);
	   GPIO_EnableInt(PF, 2, modeArray[mode]);
	   GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, GPIO_DBCLKSEL_32);
	   GPIO_ENABLE_DEBOUNCE(PF, BIT2);
	   NVIC_SetPriority(GPCDF_IRQn, 2);
	   NVIC_EnableIRQ(GPCDF_IRQn);
	break;
    case 2://PC4
    	SYS->ALT_MFP &= ~SYS_ALT_MFP_PC4_UART0_RXD;
		GPIO_SetMode(PC, BIT4, GPIO_PMD_INPUT);
		GPIO_EnableInt(PC, 4, modeArray[mode]);
		GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, GPIO_DBCLKSEL_32);
    	GPIO_ENABLE_DEBOUNCE(PC, BIT4);
		NVIC_SetPriority(GPCDF_IRQn, 2);
		NVIC_EnableIRQ(GPCDF_IRQn);
	break;
    case 3://PC5
     	SYS->ALT_MFP &= ~SYS_ALT_MFP_PC5_UART0_TXD;
		GPIO_SetMode(PC, BIT5, GPIO_PMD_INPUT);
		GPIO_EnableInt(PC, 5, modeArray[mode]);
		GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, GPIO_DBCLKSEL_32);
    	GPIO_ENABLE_DEBOUNCE(PC, BIT5);
		NVIC_SetPriority(GPCDF_IRQn, 2);
		NVIC_EnableIRQ(GPCDF_IRQn);
    }
  }
}

void detachInterrupt(uint8_t interruptNum) {
  if(interruptNum < EXTERNAL_NUM_INTERRUPTS) {
    switch (interruptNum) {
		case 0://PF3
			GPIO_SetMode(PF, BIT3, GPIO_PMD_QUASI);
			GPIO_DisableInt(PF, 3);
		break;
		case 1://PF2
			GPIO_SetMode(PF, BIT2, GPIO_PMD_QUASI);
			GPIO_DisableInt(PF, 2);
		break;
		case 2://PB0
			GPIO_SetMode(PC, BIT4, GPIO_PMD_INPUT);
			GPIO_DisableInt(PC, 4);
		break;
		case 3://PB1
			GPIO_SetMode(PC, BIT5, GPIO_PMD_INPUT);
			GPIO_DisableInt(PC, 5);
		break;
		}
    intFunc[interruptNum] = 0;
  }
}
#if 0
void GPAB_IRQHandler(void)
{
    /* To check if PB.0 INT2 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT0)){//int2
        GPIO_CLR_INT_FLAG(PB, BIT0);
        //Serial.print("PB0 INT occurred.\r\n");
		if(intFunc[2]){
			intFunc[2]();
		}
    }else if(GPIO_GET_INT_FLAG(PB, BIT1)){//int3
        GPIO_CLR_INT_FLAG(PB, BIT1);
        //Serial.print("PB1 INT occurred.\r\n");
		if(intFunc[3]){
			intFunc[3]();
		}
    }else{
        /* Un-expected interrupt. Just clear all PA, PB interrupts */
        PA->ISRC = PA->ISRC;
        PB->ISRC = PB->ISRC;
        Serial.print("Un-expected interrupts.\r\n");
    }
}
#endif
/**
 * @brief       GPIO PC/PD/PF IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PC/PD/PF default IRQ, declared in startup_NUC123.s.
 */
void GPCDF_IRQHandler(void)
{
    /* To check if PF.3 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PF, BIT3)){ // int0
        GPIO_CLR_INT_FLAG(PF, BIT3);
        //Serial.print("PF3 INT occurred.\r\n");
		if(intFunc[0]){
			intFunc[0]();
		}
    }else if(GPIO_GET_INT_FLAG(PF, BIT2)){ // int1
        GPIO_CLR_INT_FLAG(PF, BIT2);
        //Serial.print("PF2 INT occurred.\r\n");
		if(intFunc[1]){
			intFunc[1]();
		}
    }else if(GPIO_GET_INT_FLAG(PC, BIT4)){//int2
        GPIO_CLR_INT_FLAG(PC, BIT4);
        //Serial.print("PC4 INT occurred.\r\n");
		if(intFunc[2]){
			intFunc[2]();
		}
    }else if(GPIO_GET_INT_FLAG(PC, BIT5)){//int3
        GPIO_CLR_INT_FLAG(PC, BIT5);
        //Serial.print("PC5 INT occurred.\r\n");
		if(intFunc[3]){
			intFunc[3]();
		}
    }else{
        /* Un-expected interrupt. Just clear all PC, PD and PF interrupts */
        PC->ISRC = PC->ISRC;
        PD->ISRC = PD->ISRC;
        PF->ISRC = PF->ISRC;
        Serial.print("Un-expected interrupts.\r\n");
    }
}

