// For NUC123:
#define _useTimer0
#define _useTimer1
//#define _useTimer2


#if defined (_useTimer0)
#define TC_FOR_TIMER0       TC0
#define CHANNEL_FOR_TIMER0  0
#define ID_TC_FOR_TIMER0    ID_TC0
#define IRQn_FOR_TIMER0     TC0_IRQn
#define HANDLER_FOR_TIMER0  TC0_Handler
#endif
#if defined (_useTimer1)
#define TC_FOR_TIMER1       TC1
#define CHANNEL_FOR_TIMER1  1
#define ID_TC_FOR_TIMER1    ID_TC1
#define IRQn_FOR_TIMER1     TC1_IRQn
#define HANDLER_FOR_TIMER1  TC1_Handler
#endif
#if defined (_useTimer2)
#define TC_FOR_TIMER2       TC2
#define CHANNEL_FOR_TIMER2  2
#define ID_TC_FOR_TIMER2    ID_TC2
#define IRQn_FOR_TIMER2     TC2_IRQn
#define HANDLER_FOR_TIMER2  TC2_Handler
#endif
#if defined (_useTimer3)
#define TC_FOR_TIMER3       TC3
#define CHANNEL_FOR_TIMER3  3
#define ID_TC_FOR_TIMER3    ID_TC3
#define IRQn_FOR_TIMER3     TC3_IRQn
#define HANDLER_FOR_TIMER3  TC3_Handler
#endif

typedef enum { _timer0=0, _timer1, _Nbr_16timers } timer16_Sequence_t ;

