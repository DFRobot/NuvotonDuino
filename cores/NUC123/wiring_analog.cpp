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
#include "nuc123.h"

#ifdef __cplusplus
extern "C" {
#endif

#define   UNUSED_TICK     0xffffffff

typedef struct _TickPin{
	volatile uint32_t tick;
	volatile uint32_t  pin;
}tTickPin;
tTickPin tickPin[3] = {{UNUSED_TICK, 0},{UNUSED_TICK, 0},{UNUSED_TICK, 0}};
tTickPin tickPinBuf[3];
static  uint8_t softPWMTickNum= 0;
static  uint8_t softPWMTickNumBuf= 0;
static 	uint8_t softPWMTimerInit = 0;
static volatile uint8_t softPWMSetFlag = 0; //À˘”–pin «∑Ò±ª÷ÿ–¬…Ë÷√

static uint32_t currentTick = 0,currentTickNum = 0;
static uint32_t currentPri= 1;


static int _readResolution = 10;
static int _writeResolution = 8;
volatile int g_u32AdcIntFlag = 0;

static uint8_t PWMEnabled = 0;
static uint8_t pinEnabled[PWM_MAX_COUNT]={0};
static uint32_t fixValue[PWM_MAX_COUNT]={0};


void sort_tick(tTickPin* p)
{
  uint8_t i, j;
  tTickPin temp;
  for(i = 0; i < 2; i++)
    for(j = i + 1; j < 3 ; j++)
        if (p[i].tick > p[j].tick){
			
          temp = p[i];
          p[i] = p[j];
          p[j] = temp;
		  
        }
}

void calc_tick(uint32_t ulPin,uint32_t ulValue)
{
	uint8_t  i,j;
	uint32_t tick;
	uint8_t  ticknum;
	if(softPWMSetFlag == 0 ){	//Œ¥±ª÷ÿ–¬≈≈–Ú
		for(i=0;i<3;i++)
			tickPinBuf[i] = tickPin[i];
		softPWMTickNumBuf =  softPWMTickNum;
	}
	
	if((ulValue >= 254) || (ulValue <= 1)){
		for(i=0;i<3;i++){
			if(tickPinBuf[i].pin & (1 << ulPin)){		//∏√Õ®µ¿÷Æ«∞“—‘⁄ ‰≥ˆPWM£
				tickPinBuf[i].pin &= ~(1 << ulPin);		//»•≥˝TICK ”ÎPIN µƒ∂‘”¶πÿœµ
				if(tickPinBuf[i].pin == 0){				//“—ŒﬁÕ®µ¿ ‰≥ˆ∏√TICK£¨»•≥˝∏√TICK
					tickPinBuf[i].tick = UNUSED_TICK;
					if(softPWMTickNumBuf > 0){
						softPWMTickNumBuf--;
						sort_tick(tickPinBuf);//TICK÷ÿ–¬≈≈–Ú
						softPWMSetFlag = 1;
					}
				}
			break;
			}
		}
		if(ulValue >= 254){
			digitalWrite(ulPin,HIGH);
		}else{
			digitalWrite(ulPin,LOW);
		}
		return;											 
	}

	
	tick = (ulValue * 24000) >> 8;
	if(tick < 0x02)
		tick = 0x02;
	for(i=0;i<3;i++){
		if(tickPinBuf[i].pin & (1 << ulPin)){	 	  	 //≤Èø¥µ±«∞µƒpin «∑Ò“—‘⁄ ‰≥ˆPWM
			if(tickPinBuf[i].tick == tick){     	  	 //pin∂‘”¶µƒtick  Œ¥∏ƒ±‰
				return;
			}else{ 
				tickPinBuf[i].pin &= ~(1 << ulPin); 	 //µ±«∞µƒpin ∂‘”¶µƒ pick “—∏ƒ±‰°¢«Â≥˝÷Æ«∞pin ”Î tick µƒ∂‘”¶πÿœµ
				if(tickPinBuf[i].pin == 0){	  	  		 //“—√ª”–Õ®µ¿ ‰≥ˆ∏√TICK£¨«Â≥˝∏√TICK
					tickPinBuf[i].tick = UNUSED_TICK;
					if(softPWMTickNumBuf > 0)
						softPWMTickNumBuf--;
					sort_tick(tickPinBuf);//
				}
				break;
			}
		}
	}
	for(i=0;i<3;i++){							  		 //≤Èø¥ «∑Ò”–œ‡Õ¨µƒTICK‘⁄‘À––
		if(tickPinBuf[i].tick == tick){
			tickPinBuf[i].pin |= 1 << ulPin;
			softPWMSetFlag = 1;
			return;
		}
	}

	for(i=0;i<3;i++){						      		//√ª”–œ‡Õ¨µƒtick‘⁄‘À––£¨≤Â»Î“ª∏ˆ–¬µƒtick
		if(tickPinBuf[i].tick == UNUSED_TICK){
			tickPinBuf[i].tick = tick;
			tickPinBuf[i].pin |= 1 << ulPin;
			softPWMTickNumBuf++;
			sort_tick(tickPinBuf);
			softPWMSetFlag = 1;
			break;
		}
	}
}

void SoftPWM_Handler(TIMER_T* t)
{
	uint8_t j;
	currentTick = t->TCMPR;
	if(currentTick != 24000*currentPri){ //±»Ωœ≤˙…˙µƒ÷–∂œ
		if(tickPin[currentTickNum].pin & BIT3)
			digitalWrite(3,LOW);
		if(tickPin[currentTickNum].pin & BIT11){
			digitalWrite(11,LOW);
		}
		if(tickPin[currentTickNum].pin & BIT13)
			digitalWrite(13,LOW);
		
		if(currentTickNum == softPWMTickNum - 1){//À˘”––Ë“™÷¥––µƒ÷–∂œ»´≤ø÷¥––ÕÍ≥…£¨µ»¥˝÷‹∆⁄Ω· ¯
			t->TCMPR = 24000*currentPri;
			currentTickNum = 0;
		}else{									//…Ë÷√–¬µƒPWM÷‹∆⁄÷’÷π ±º‰
			currentTickNum++;
			t->TCMPR = tickPin[currentTickNum].tick + (currentPri - 1)* 24000;	
		}
		
	}else{										//÷‹∆⁄Ω· ¯≤˙…˙µƒ÷–∂œ
		if(softPWMSetFlag){//±ª÷ÿ–¬≈≈–Ú
			softPWMTickNum = softPWMTickNumBuf ;
			for(j=0;j<3;j++)
				tickPin[j] = tickPinBuf[j];
			softPWMSetFlag = 0;
			if(softPWMTickNum == 0){
				softPWMTimerInit = 0;
				TIMER_Stop(TIMER3);
				TIMER3->TDR   = 0;
				currentPri = 1;
				return;
			}
		}
		for(j=0;j<softPWMTickNum;j++){
			
			if(tickPin[j].pin & BIT3){
				digitalWrite(3,HIGH);
			}
			if(tickPin[j].pin & BIT11){
				digitalWrite(11,HIGH);
			}
			if(tickPin[j].pin & BIT13){
				digitalWrite(13,HIGH);
			}
		}
		
		t->TCMPR = tickPin[0].tick + 24000*currentPri;
		currentPri++;
		if(currentPri == 700)
			currentPri = 1;
	}
}


void TMR3_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER3) == 1){
		
		/* Clear Timer0 time-out interrupt flag */
		TIMER_ClearIntFlag(TIMER3);
		SoftPWM_Handler(TIMER3);
    }
}

void ADC_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* clear the A/D conversion flag */
}

void analogReadResolution(int res) {
	_readResolution = res;
}

void analogWriteResolution(int res) {
	_writeResolution = res;
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) {
	if (from == to)
		return value;
	if (from > to)
		return (value >> (from-to));
	else
		return (value << (to-from));
}

eAnalogReference analog_reference = AR_DEFAULT;

void analogReference(eAnalogReference ulMode)
{
	analog_reference = ulMode;
}

uint32_t analogRead(uint32_t ulPin)
{

	uint32_t ulValue = 0;  

	if(ulPin>ADC_MAX_COUNT || ADC_Desc[ulPin].A==NULL) return 0;  	  
	SYS_UnlockReg();
	/* Configure the GPD0 - GPD3 ADC analog input pins */
	SYS->GPD_MFP &= ~(1UL<<ulPin) ;
	SYS->GPD_MFP |= (1UL<<ulPin);
	SYS->ALT_MFP1 &= ~(1UL<<(ulPin+16));
	SYS->ALT_MFP1 |= (1UL<<(ulPin+16)); 
	SYS_LockReg();  	
	/* Disable the digital input path to avoid the leakage current. */
	GPIO_DISABLE_DIGITAL_PATH(PD, (1UL<<ulPin));
	//GPIO_ENABLE_DIGITAL_PATH(GPIO_Desc[ADC_Desc[ulPin].pintype.num].P,GPIO_Desc[ADC_Desc[ulPin].pintype.num].bit);

	// Enable channel 
	//ADC_Open(ADC_Desc[ulPin].A, ADC_INPUT_MODE_SINGLE_END, ADC_OPERATION_MODE_SINGLE, (1<<ADC_Desc[ulPin].ch));
  	ADC_Open(ADC, NULL, ADC_ADCR_ADMD_SINGLE, (1UL << ulPin));

	// Power on ADC
	ADC_POWER_ON(ADC_Desc[ulPin].A);

	ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
	/* Enable the ADC interrupt */
	
	/* Reset the ADC interrupt indicator and Start A/D conversion */
	// Wait for busy of conversion
  	while(ADC_IS_BUSY(ADC_Desc[ulPin].A));
  	// Start for conversion	
  	ADC_START_CONV(ADC_Desc[ulPin].A);
  		
	// Wait for end of conversion
	while(!ADC_GET_INT_FLAG(ADC_Desc[ulPin].A,ADC_ADF_INT));
	
	
	// Clear ADC flag
	ADC_CLR_INT_FLAG(ADC_Desc[ulPin].A,ADC_ADF_INT);
	
	// Read the value
	ulValue = ADC_GET_CONVERSION_DATA(ADC_Desc[ulPin].A,ADC_Desc[ulPin].ch);	
	//ulValue = mapResolution(ulValue, 12, _readResolution);
  
  	//GPIO_DISABLE_DIGITAL_PATH(GPIO_Desc[ADC_Desc[ulPin-1].pintype.num].P,GPIO_Desc[ADC_Desc[ulPin].pintype.num-1].bit);
  
  	// Close ADC
  	ADC_Close(ADC_Desc[ulPin].A);
  	return ulValue;	
}

// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
//static uint8_t PWMEnabled = 0;
//static uint8_t pinEnabled[PWM_MAX_COUNT]={0};
//static uint32_t fixValue[PWM_MAX_COUNT]={0};
#define MIN(a,b)		( (a) > (b) ? b:a )
void configPWM(uint8_t pwmNo)
{
	
}
void analogWrite(uint32_t ulPin, uint8_t ulValue) {
	static uint8_t start[4]={0};
	uint8_t hardPWM=0,pwmCh;
	uint32_t duty;
	int i;
	#ifdef USE_BoardToPin
	if(ulPin > BoardToPin_MAX_COUNT) return;
	#endif
	for(i=0;i<4;i++){
		if(PWM_Desc[i].pin == ulPin){
			hardPWM = 1;
			pwmCh = PWM_Desc[i].ch;
			break;
		}
	}

	if(hardPWM){
		if(!start[pwmCh]){
			
			SYS_UnlockReg();
			SYS->GPA_MFP |= 1UL<<(12+pwmCh);
			if((pwmCh == 0) || (pwmCh == 1)){
				CLK_EnableModuleClock(PWM01_MODULE);
				CLK_SetModuleClock(PWM01_MODULE, CLK_CLKSEL1_PWM01_S_HXT, 0);
			}else{
				CLK_EnableModuleClock(PWM23_MODULE);
				CLK_SetModuleClock(PWM23_MODULE, CLK_CLKSEL1_PWM01_S_HXT, 0);
			}		
			SYS_LockReg();
			
			// PWM0 frequency is 500Hz, duty ulValue%,
    		PWM_ConfigOutputChannel(PWMA, pwmCh, 500, ulValue);
   			PWM_EnableOutput(PWMA, 1<<pwmCh);
			PWM_Start(PWMA, 1<<pwmCh);
			fixValue[pwmCh]!=ulValue;
			start[pwmCh] = 1;
		}else if(fixValue[pwmCh]!=ulValue){
			PWM_ConfigOutputChannel(PWM_Desc[pwmCh].P,PWM_Desc[pwmCh].ch,PWM_Desc[pwmCh].freq,ulValue);
			fixValue[pwmCh]=ulValue;
		}
	}else{
		//softPWM/////////////////////////////////
		if(ulPin != 3 && ulPin != 11 && ulPin != 13)
			return;
		calc_tick(ulPin,ulValue);
		if(tickPinBuf[0].tick == UNUSED_TICK){
			return;
		}
		if(softPWMTimerInit == 0){
			for(i=0;i<3;i++)
				tickPin[i] = tickPinBuf[i];
			softPWMTickNum = softPWMTickNumBuf;
			SYS_UnlockReg();
		    CLK_EnableModuleClock(TMR3_MODULE);
			CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3_S_HXT  , 0);
		    SYS_LockReg();
			
		  	TIMER_Open(TIMER3, TIMER_CONTINUOUS_MODE,500); 
		    TIMER_SET_PRESCALE_VALUE(TIMER3, 0);
		    TIMER_SET_CMP_VALUE(TIMER3, tickPin[0].tick); 
		    TIMER_EnableInt(TIMER3);
		    /* Enable Timer1 NVIC */
		    NVIC_EnableIRQ(TMR3_IRQn);
		    TIMER_Start(TIMER3);
			softPWMTimerInit = 1;
		}
	}
}

#ifdef __cplusplus
}
#endif

