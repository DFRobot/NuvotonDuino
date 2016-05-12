

#ifndef __DELAY_H
#define __DELAY_H

#ifdef __cplusplus
extern "C" {
#endif

extern void lfclk_config(void);

extern uint32_t millis( void );
extern uint32_t micros( void );
extern void delay( uint32_t ms ) ;
extern void delayMicroseconds( uint32_t us );
extern void rtc_timer_init();

#ifdef __cplusplus
}
#endif

#endif 
