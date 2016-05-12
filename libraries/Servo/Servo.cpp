#include <Arduino.h> 
#include <Servo.h>

#define usToTicks(_us)    (( clockCyclesPerMicrosecond() * _us))     // converts microseconds to tick
#define ticksToUs(_ticks) (( (unsigned)_ticks)/ clockCyclesPerMicrosecond() ) // converts from ticks back to microseconds


#define TRIM_DURATION       2                               // compensation ticks to trim adjust for digitalWrite delays

static servo_t servos[MAX_SERVOS];                          // static array of servo structures

uint8_t ServoCount = 0;                                     // the total number of attached servos

static volatile int8_t Channel[_Nbr_16timers ]={-1,-1};             // counter for the servo being pulsed for each timer (or -1 if refresh interval)

// convenience macros
#define SERVO_INDEX_TO_TIMER(_servo_nbr) ((timer16_Sequence_t)(_servo_nbr / SERVOS_PER_TIMER)) // returns the timer controlling this servo
#define SERVO_INDEX_TO_CHANNEL(_servo_nbr) (_servo_nbr % SERVOS_PER_TIMER)       // returns the index of the servo on this timer
#define SERVO_INDEX(_timer,_channel)  ((_timer*SERVOS_PER_TIMER) + _channel)     // macro to access servo index by timer and channel
#define SERVO(_timer,_channel)  (servos[SERVO_INDEX(_timer,_channel)])            // macro to access servo class by timer and channel

#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)  // minimum value in uS for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)  // maximum value in uS for this servo 

/************ static functions common to all instances ***********************/


//timer16_Sequence_t timer;

//------------------------------------------------------------------------------
/// Interrupt handler for the TC0 channel 1.
//------------------------------------------------------------------------------
void Servo_Handler(timer16_Sequence_t timer, TIMER_T *pTc, uint8_t channel);
#if defined (_useTimer0)
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1){
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);
		Servo_Handler(_timer0, TIMER0, CHANNEL_FOR_TIMER0);
    }
}

#endif
#if defined (_useTimer1)
void TMR1_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER1) == 1){
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER1);
		Servo_Handler(_timer1, TIMER1, CHANNEL_FOR_TIMER1);
    }
}
#endif
#if defined (_useTimer2)
void TMR2_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER2) == 1){
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER2);
		Servo_Handler(_timer1, TIMER2, CHANNEL_FOR_TIMER2);
    }
}
#endif
#if defined (_useTimer3)
void TMR3_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER3) == 1){
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER3);
		Servo_Handler(_timer3, TIMER3, CHANNEL_FOR_TIMER3);
    }
}
#endif

void Servo_Handler(timer16_Sequence_t timer, TIMER_T *t, uint8_t channel)
{
    if (Channel[timer] < 0) {//ÖØÆô¶¨Ê±Æ÷
        t->TCSR |= TIMER_TCSR_CRST_Msk;
		TIMER_Start(t);
	} else {
        if (SERVO_INDEX(timer,Channel[timer]) < ServoCount && SERVO(timer,Channel[timer]).Pin.isActive == true) {
            digitalWrite(SERVO(timer,Channel[timer]).Pin.nbr, LOW); // pulse this channel low if activated
        }
    }

    Channel[timer]++;    // increment to the next channel
    if( SERVO_INDEX(timer,Channel[timer]) < ServoCount && Channel[timer] < SERVOS_PER_TIMER) {
        //tc->TC_CHANNEL[channel].TC_RA = tc->TC_CHANNEL[channel].TC_CV + SERVO(timer,Channel[timer]).ticks;
        t->TCMPR = t->TDR + SERVO(timer,Channel[timer]).ticks;
        if(SERVO(timer,Channel[timer]).Pin.isActive == true) {    // check if activated
            digitalWrite( SERVO(timer,Channel[timer]).Pin.nbr,HIGH); // its an active channel so pulse it high   
        }
    } else { 
        if( (t->TDR + 4) < usToTicks(REFRESH_INTERVAL)){
			t->TCMPR = (unsigned int)usToTicks(REFRESH_INTERVAL);
		}else{
			t->TCMPR = t->TDR + 4;
		}
        Channel[timer] = -1; // this will get incremented at the end of the refresh period to start again at the first channel
    }
}

static void initISR(timer16_Sequence_t timer)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
	
#if defined (_useTimer0)
		if (timer == _timer0){
			CLK_EnableModuleClock(TMR0_MODULE);
			CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HXT, 0);
			TIMER_Open(TIMER0, TIMER_CONTINUOUS_MODE, 1000);
			TIMER_EnableInt(TIMER0);
			NVIC_EnableIRQ(TMR0_IRQn);
			TIMER_Start(TIMER0);
		}
#endif
#if defined (_useTimer1)
		if (timer == _timer1){
			CLK_EnableModuleClock(TMR1_MODULE);
			CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HXT, 0);
			TIMER_Open(TIMER1, TIMER_CONTINUOUS_MODE, 1000);
			TIMER_EnableInt(TIMER1);
			NVIC_EnableIRQ(TMR1_IRQn);
			TIMER_Start(TIMER1);
		}
#endif    
#if defined (_useTimer2)
		if (timer == _timer2){  // tone
			CLK_EnableModuleClock(TMR2_MODULE);
			CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2_S_HXT, 0);
			TIMER_Open(TIMER2, TIMER_CONTINUOUS_MODE, 1000);
			TIMER_EnableInt(TIMER2);
			NVIC_EnableIRQ(TMR2_IRQn);
			TIMER_Start(TIMER2);
		}
#endif
#if defined (_useTimer3)
		if (timer == _timer3){ //softPWM
			CLK_EnableModuleClock(TMR3_MODULE);
			CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3_S_HXT, 0);
			TIMER_Open(TIMER3, TIMER_CONTINUOUS_MODE, 1000);
			TIMER_EnableInt(TIMER3);
			NVIC_EnableIRQ(TMR3_IRQn);
			TIMER_Start(TIMER3);
		}
#endif
    /* Lock protected registers */
    SYS_LockReg();
} 

static void finISR(timer16_Sequence_t timer)
{
#if defined (_useTimer0)
	TIMER_Stop(TIMER0);
	NVIC_DisableIRQ(TMR0_IRQn);
	TIMER_DisableInt(TIMER0);
	TIMER_Close(TIMER0);
	CLK_DisableModuleClock(TMR0_MODULE);
#endif
#if defined (_useTimer1)
	TIMER_Stop(TIMER1);
	NVIC_DisableIRQ(TMR1_IRQn);
	TIMER_DisableInt(TIMER1);
	TIMER_Close(TIMER1);
	CLK_DisableModuleClock(TMR1_MODULE);

#endif    
#if defined (_useTimer2)
	TIMER_Stop(TIMER2);
	NVIC_DisableIRQ(TMR2_IRQn);
	TIMER_DisableInt(TIMER2);
	TIMER_Close(TIMER2);
	CLK_EnableModuleClock(TMR2_MODULE);
#endif    
#if defined (_useTimer3)
	TIMER_Stop(TIMER3);
	NVIC_DisableIRQ(TMR3_IRQn);
	TIMER_DisableInt(TIMER3);
	TIMER_Close(TIMER3);
	CLK_DisableModuleClock(TMR3_MODULE);

#endif      
}


static boolean isTimerActive(timer16_Sequence_t timer)
{
  // returns true if any servo is active on this timer
  for(uint8_t channel=0; channel < SERVOS_PER_TIMER; channel++) {
    if(SERVO(timer,channel).Pin.isActive == true)
      return true;
  }
  return false;
}

/****************** end of static functions ******************************/

Servo::Servo()
{
  if (ServoCount < MAX_SERVOS) {
    this->servoIndex = ServoCount++;                    // assign a servo index to this instance
	servos[this->servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH);   // store default values
  } else {
    this->servoIndex = INVALID_SERVO;  // too many servos 
  }
}

uint8_t Servo::attach(int pin)
{
  return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t Servo::attach(int pin, int min, int max)
{
  timer16_Sequence_t timer;
  
  if (this->servoIndex < MAX_SERVOS) {
    pinMode(pin, OUTPUT);                                   // set servo pin to output
    servos[this->servoIndex].Pin.nbr = pin;  
    // todo min/max check: abs(min - MIN_PULSE_WIDTH) /4 < 128 
    this->min  = (MIN_PULSE_WIDTH - min)/4; //resolution of min/max is 4 uS
    this->max  = (MAX_PULSE_WIDTH - max)/4; 
    // initialize the timer if it has not already been initialized 
    timer = SERVO_INDEX_TO_TIMER(servoIndex);
    if (isTimerActive(timer) == false) {
      initISR(timer);    
    }
    servos[this->servoIndex].Pin.isActive = true;  // this must be set after the check for isTimerActive
  }
  return this->servoIndex;
}

void Servo::detach()  
{
  timer16_Sequence_t timer;

  servos[this->servoIndex].Pin.isActive = false;  
  timer = SERVO_INDEX_TO_TIMER(servoIndex);
  if(isTimerActive(timer) == false) {
    finISR(timer);
  }
}

void Servo::write(int value)
{  
  // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
  if (value < MIN_PULSE_WIDTH)
  {  
    if (value < 0)
      value = 0;
    else if (value > 180)
      value = 180;
    value = map(value, 0, 180, SERVO_MIN(), SERVO_MAX());
  }
  writeMicroseconds(value);
}

void Servo::writeMicroseconds(int value)
{
  // calculate and store the values for the given channel
  byte channel = this->servoIndex;
  if( (channel < MAX_SERVOS) )   // ensure channel is valid
  {  
    if (value < SERVO_MIN())          // ensure pulse width is valid
      value = SERVO_MIN();
    else if (value > SERVO_MAX())
      value = SERVO_MAX();   

    value = value - TRIM_DURATION;
    value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead
    servos[channel].ticks = value;  
  }
}

int Servo::read() // return the value as degrees
{
  return map(readMicroseconds()+1, SERVO_MIN(), SERVO_MAX(), 0, 180);
}

int Servo::readMicroseconds()
{
  unsigned int pulsewidth;
  if (this->servoIndex != INVALID_SERVO)
    pulsewidth = ticksToUs(servos[this->servoIndex].ticks)  + TRIM_DURATION;
  else 
    pulsewidth  = 0;

  return pulsewidth;   
}

bool Servo::attached()
{
  return servos[this->servoIndex].Pin.isActive;
}

