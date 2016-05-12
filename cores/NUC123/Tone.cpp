/* Tone.cpp

  A Tone Generator Library

  Written by Brett Hagman

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

Version Modified By Date     Comments
------- ----------- -------- --------
0001    B Hagman    09/08/02 Initial coding
0002    B Hagman    09/08/18 Multiple pins
0003    B Hagman    09/08/18 Moved initialization from constructor to begin()
0004    B Hagman    09/09/26 Fixed problems with ATmega8
0005    B Hagman    09/11/23 Scanned prescalars for best fit on 8 bit timers
                    09/11/25 Changed pin toggle method to XOR
                    09/11/25 Fixed timer0 from being excluded
0006    D Mellis    09/12/29 Replaced objects with functions
0007    M Sproul    10/08/29 Changed #ifdefs from cpu to register
0008    S Kanemoto  12/06/22 Fixed for Leonardo by @maris_HY
0009    J Reucker   15/04/10 Issue #292 Fixed problems with ATmega8 (thanks to Pete62)
0010    jipp        15/04/13 added additional define check #2923
*************************************************/

//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
#include "Arduino.h"
#include "pins_arduino.h"



// timerx_toggle_count:
//  > 0 - duration specified
//  = 0 - stopped
//  < 0 - infinitely (until stop() method called, or new play() called)


volatile long timer2_toggle_count;

#define AVAILABLE_TONE_PINS 1
//#define USE_TIMER2

static uint8_t tone_pins =  255 ;





static int8_t toneBegin(uint8_t _pin)
{
  int8_t _timer = -1;

  
  // search for an unused timer.
 
    if (tone_pins == 255) {
      tone_pins = _pin;
	  _timer = 2;
	  return _timer;
    }
  return _timer;
}



// frequency (in hertz) and duration (in milliseconds).
void tone(unsigned long _pin, unsigned short frequency, unsigned long duration)
{
	int8_t _timer;
    
	_timer = toneBegin(_pin); 
	if (_timer >= 0)
	{
		pinMode(_pin, OUTPUT); 
		if (_timer == 2)
		{  
			SYS_UnlockReg();
			CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2_S_HXT , 0);
		    SYS_LockReg(); 
			if(frequency > 1500 ){
				frequency = 120;
			} 
			TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 0xfffff0); 
			TIMER2->TCMPR = 12000000/frequency / 2;
			NVIC_SetPriority(TMR2_IRQn, 3);
			TIMER_EnableInt(TIMER2);
			NVIC_EnableIRQ(TMR2_IRQn);
			
		}

		// Calculate the toggle count
		if (duration > 0){
			timer2_toggle_count = 2 * frequency * duration / 1000;
		}
		else{
			timer2_toggle_count = -1;
		}
		TIMER_Start(TIMER2);  
		
	}
}



// XXX: this function only works properly for timer 2 (the only one we use
// currently).  for the others, it should end the tone, but won't restore
// proper PWM functionality for the timer.
void disableTimer(uint8_t _timer)
{
	switch (_timer)
	{
	case 2:
		TIMER_Stop(TIMER2);	
	  	break;
	default:
		break;
	}
}


void noTone(uint8_t _pin)
{
	int8_t _timer = -1;

	
	if (tone_pins == _pin) {
		tone_pins = 255;
		_timer = 2;
	}
	

	disableTimer(_timer);

	digitalWrite(_pin, 0);
}





void TMR2_IRQHandler(void)
{
	static uint8_t toggleFlag = 0x00;
	
    if(TIMER_GetIntFlag(TIMER2) == 1)
    {
        /* Clear Timer3 time-out interrupt flag */
	
        TIMER_ClearIntFlag(TIMER2);
		toggleFlag = !toggleFlag; 
		if (timer2_toggle_count > 0)
		{
			// toggle the pin
			if(toggleFlag){
				
				digitalWrite(tone_pins,HIGH);
			}else{
				
				digitalWrite(tone_pins,LOW);
			}
			if (timer2_toggle_count > 0)
				timer2_toggle_count--;
			
		}
		else{
			noTone(tone_pins);
		}
		
	}
	
}


