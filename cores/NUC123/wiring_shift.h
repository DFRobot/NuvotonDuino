

#ifndef _WIRING_SHIFT_
#define _WIRING_SHIFT_

#ifdef __cplusplus
 extern "C" {
#endif


extern uint32_t shiftIn( uint32_t ulDataPin, uint32_t ulClockPin, uint32_t ulBitOrder ) ;

extern void shiftOut( uint32_t ulDataPin, uint32_t ulClockPin, uint32_t ulBitOrder, uint32_t ulVal ) ;


#ifdef __cplusplus
}
#endif

#endif /* _WIRING_SHIFT_ */
