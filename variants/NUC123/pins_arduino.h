#ifndef __PINS_ARDUINO_H
#define __PINS_ARDUINO_H
#include "Arduino.h"

#define USE_BoardToPin
#define BoardToPin_MAX_COUNT 32
#define ADC_MAX_COUNT        6



#define  SS     43
#define  MOSI   42
#define  MISO   40
#define  SCK    41

#define  SDA    14
#define  SCL    15

#define  A0     0
#define  A1     1
#define  A2     2
#define  A3     3
#define  A4     4
#define  A5     5

#define PWM_MAX_COUNT  8
#define I2C_MAX_COUNT  1
#define SPI_MAX_COUNT  1
#define UART_MAX_COUNT 2

#define RX_LED 17
#define TX_LED 32

typedef struct _PinType
{
	uint32_t Msk;  
    uint32_t Msk1;
} PinType;


typedef struct _PinDescription
{
	uint32_t MFP;	
	uint32_t MFP_MASK;  
	uint32_t ALT;
	uint32_t ALT_MASK;
} PinDescription;

typedef struct _GPIOPinDescription
{
	GPIO_T *P;
	uint32_t bit;
  PinDescription Pin;
} GPIOPinDescription;

typedef struct _ADCPinDescription
{
    ADC_T *A;
	uint32_t module;
	uint32_t ch;
	PinType pintype;
} ADCPinDescription;

typedef struct _PWMPinDescription
{
	PWM_T *P;
	uint32_t module;
	IRQn_Type irq;
	uint32_t ch;
	uint32_t freq;
	uint32_t pin;
	PinType pintype;
} PWMPinDescription;

extern const GPIOPinDescription GPIO_Desc[];
extern const ADCPinDescription ADC_Desc[];
extern const PWMPinDescription PWM_Desc[];
extern const u32  digital_pin_to_port[];

extern const u32 digital_pin_to_bit_mask[];
extern volatile u32 *digital_pin_to_input[];
#endif
