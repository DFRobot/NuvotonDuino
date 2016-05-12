/*
 *  Interrupt and PWM utilities for 16 bit Timer1 on ATmega168/328
 *  Original code by Jesse Tane for http://labs.ideo.com August 2008
 *  Modified March 2009 by Jér?me Despatis and Jesse Tane for ATmega328 support
 *  Modified June 2009 by Michael Polli and Jesse Tane to fix a bug in setPeriod() which caused the timer to stop
 *  Modified Oct 2009 by Dan Clemens to work with timer1 of the ATMega1280 or Arduino Mega
 *  Modified April 2012 by Paul Stoffregen
 *  Modified again, June 2014 by Paul Stoffregen
 *
 *  This is free software. You can redistribute it and/or modify it under
 *  the terms of Creative Commons Attribution 3.0 United States License. 
 *  To view a copy of this license, visit http://creativecommons.org/licenses/by/3.0/us/ 
 *  or send a letter to Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA.
 *
 */

#include "TimerOne.h"

TimerOne Timer1;              // preinstatiate

unsigned short TimerOne::pwmPeriod = 0;
unsigned char TimerOne::clockSelectBits = 0;
void (*TimerOne::isrCallback)() = NULL;

void TMR1_IRQHandler(void)
{
	//static int state=0;
    if(TIMER_GetIntFlag(TIMER1) == 1){
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER1);
		if(Timer1.isrCallback)
			Timer1.isrCallback();
		//digitalWrite(13, state);
		//state = !state;
		//{//clk = 12 000 000 HZ
		//	uint32_t u32Clk = TIMER_GetModuleClock(TIMER1);
		//	Serial.print("Timer1Clock=");Serial.println(u32Clk);
    	//}
	}
}

