/*
  Copyright (c) 2012 Arduino.  All right reserved.

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

#ifndef Arduino_h
#define Arduino_h

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <startup_nuc123.h>
// some libraries and sketches depend on this
// AVR stuff, assuming Arduino.h or WProgram.h
// automatically includes it..

#include "binary.h"
#include "types.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

// Includes Nuvoton CMSIS

#include "NUC123.h"

extern volatile uint8_t init_flag;	

#define countof(x) 					 (sizeof(x)/sizeof(x[0]))
#define ARRAYSIZE(x) 				 (sizeof(x)/sizeof(x[0]))

#define VARIANT_MCK         		 72000000UL	

#define clockCyclesPerMicrosecond()  ( 12000000 / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

#define digitalPinToPort(P) ( digital_pin_to_port[P] )
#define digitalPinToBitMask(P) ( digital_pin_to_bit_mask[P] )
#define digitalPinInputRegister(P) ( (volatile u32 *)( digital_pin_to_input[P] ))


void yield(void);

#include "pins_arduino.h"

#include "delay.h"
#include "wiring_digital.h"
#include "wiring_analog.h"
#include "wiring_shift.h"
#include "wiring_constants.h"

#include "pin_transform.h"

/* sketch */
extern void setup( void ) ;
extern void loop( void ) ;
extern void hook( void ) ;


#ifdef __cplusplus
} // extern "C"

#include "WCharacter.h"
#include "WString.h"
#include "Tone.h"
#include "WMath.h"
#include "HardwareSerial.h"
#include "wiring_pulse.h"
#include "interrupt.h"
#include "wuart.h"


#endif // __cplusplus




#endif // Arduino_h
