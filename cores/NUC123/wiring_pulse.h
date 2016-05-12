

#ifndef _WIRING_PULSE_
#define _WIRING_PULSE_

#ifdef __cplusplus
 extern "C" {
#endif


extern uint32_t pulseIn( uint32_t pin, uint32_t state, uint32_t timeout=1000000L ); //timeout = 1000000L
extern unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout=1000000L);


#ifdef __cplusplus
}
#endif

#endif /* _WIRING_PULSE_ */
