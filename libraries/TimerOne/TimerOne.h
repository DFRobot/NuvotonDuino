/*
 *  Interrupt and PWM utilities for 16 bit Timer1 on ATmega168/328
 *  Original code by Jesse Tane for http://labs.ideo.com August 2008
 *  Modified March 2009 by Jér?me Despatis and Jesse Tane for ATmega328 support
 *  Modified June 2009 by Michael Polli and Jesse Tane to fix a bug in setPeriod() which caused the timer to stop
 *  Modified April 2012 by Paul Stoffregen - portable to other AVR chips, use inline functions
 *  Modified again, June 2014 by Paul Stoffregen - support Teensy 3.x & even more AVR chips
 *  
 *
 *  This is free software. You can redistribute it and/or modify it under
 *  the terms of Creative Commons Attribution 3.0 United States License. 
 *  To view a copy of this license, visit http://creativecommons.org/licenses/by/3.0/us/ 
 *  or send a letter to Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA.
 *
 */

#ifndef TimerOne_h_
#define TimerOne_h_
#include "Arduino.h"

#define TIMER1_RESOLUTION 65536UL  // Timer1 is 16 bit


// Placing nearly all the code in this .h file allows the functions to be
// inlined by the compiler.  In the very common case with constant values
// the compiler will perform all calculations and simply write constants
// to the hardware registers (for example, setPeriod).


class TimerOne
{
public:
	void initialize(unsigned long microseconds=1000000){
		setPeriod(microseconds);
	}
    void setPeriod(unsigned long microseconds){
		#if 0
		const unsigned long cycles = (F_TIMER / 2000000) * microseconds;
		#endif
		SYS_UnlockReg();
		CLK_EnableModuleClock(TMR1_MODULE);
		//if(microseconds > 100000000)
		//CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_LIRC, 0);
		//else
		CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HXT, 0);
		TIMER_Interval(TIMER1, TIMER_PERIODIC_MODE, microseconds);
		//TIMER_EnableInt(TIMER1);
		//NVIC_EnableIRQ(TMR1_IRQn);
		SYS_LockReg();
		pwmPeriod = microseconds;
	}

    //****************************
    //  Run Control
    //****************************
    void start(){
		TIMER_Start(TIMER1);
    }
    void stop(){
		TIMER_Stop(TIMER1);
    }
    void restart(){
		TIMER1->TCSR |= TIMER_TCSR_CRST_Msk;
		TIMER_Start(TIMER1);
    }
    void resume(){
		TIMER_Start(TIMER1);
    }

    //****************************
    //  PWM outputs
    //****************************
    void setPwmDuty(char pin, unsigned int duty){
		int i,pwmCh=-1;
		SYS_UnlockReg();
		SYS->GPA_MFP |= 1UL<<(12+pwmCh);
		for(i=0;i<4;i++){
			if(PWM_Desc[i].pin == pin){
				pwmCh = PWM_Desc[i].ch;
				break;
			}
		}
		if(pwmCh == -1)
			return;
		if((pwmCh == 0) || (pwmCh == 1)){
			CLK_EnableModuleClock(PWM01_MODULE);
			CLK_SetModuleClock(PWM01_MODULE, CLK_CLKSEL1_PWM01_S_HXT, 0);
		}else{
			CLK_EnableModuleClock(PWM23_MODULE);
			CLK_SetModuleClock(PWM23_MODULE, CLK_CLKSEL1_PWM01_S_HXT, 0);
		}		
		SYS_LockReg();
		
		// PWM0 frequency is 500Hz, duty ulValue%,
		PWM_ConfigOutputChannel(PWMA, pwmCh, 1000000/pwmPeriod, duty>>2);
    }
    void pwm(char pin, unsigned int duty){
			int i,pwmCh=-1;
			SYS_UnlockReg();
			for(i=0;i<4;i++){
				if(PWM_Desc[i].pin == pin){
					pwmCh = PWM_Desc[i].ch;
					break;
				}
			}
			if(pwmCh == -1)
				return;
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
    		PWM_ConfigOutputChannel(PWMA, pwmCh, 1000000/pwmPeriod, duty>>2);
   			PWM_EnableOutput(PWMA, 1<<pwmCh);
			PWM_Start(PWMA, 1<<pwmCh);
    }
    void pwm(char pin, unsigned int duty, unsigned long microseconds){
		if (microseconds > 0){
			setPeriod(microseconds);
			pwm(pin, duty);
		}
    }
    void disablePwm(char pin){
		int i,pwmCh=-1;

		for(i=0;i<4;i++){
			if(PWM_Desc[i].pin == pin){
				pwmCh = PWM_Desc[i].ch;
				break;
			}
		}
		if(pwmCh == -1)
			return;
		// PWM0 frequency is 500Hz, duty ulValue%,
		PWM_Stop(PWMA, 1<<pwmCh);
		PWM_DisableOutput(PWMA, 1<<pwmCh);
    }

    void attachInterrupt(void (*isr)(void)) {
		isrCallback = isr;
		NVIC_SetPriority(TMR1_IRQn, 2);
		TIMER_EnableInt(TIMER1);
		NVIC_EnableIRQ(TMR1_IRQn);
		start();
    }
    void attachInterrupt(void (*isr)(), unsigned long microseconds) {
		if(microseconds > 0) setPeriod(microseconds);
		attachInterrupt(isr);
    }
    void detachInterrupt() {
		NVIC_DisableIRQ(TMR1_IRQn);
		TIMER_DisableInt(TIMER1);
		isrCallback = NULL;
    }
    static void (*isrCallback)( void );

  private:
    // properties
    static unsigned short pwmPeriod;
    static unsigned char clockSelectBits;

};

extern TimerOne Timer1;

#endif

