/*
  Copyright (c) 2011 Arduino.  All right reserved.

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

#ifdef __cplusplus
 extern "C" {
#endif

#include "Arduino.h"

extern void pinMode( uint32_t ulPin, uint32_t ulMode )
{
	uint32_t len;	
	#ifdef USE_BoardToPin
	if(ulPin > BoardToPin_MAX_COUNT) return;
	#endif
	
	if(GPIO_Desc[ulPin].P == NULL) return;
	if(GPIO_Desc[ulPin].Pin.ALT != NULL){
		*(volatile unsigned int*)GPIO_Desc[ulPin].Pin.ALT &= ~GPIO_Desc[ulPin].Pin.ALT_MASK;
	}
	*(volatile unsigned int*)GPIO_Desc[ulPin].Pin.MFP &= ~GPIO_Desc[ulPin].Pin.MFP_MASK;

	switch ( ulMode )
    {
        case INPUT:                        
            GPIO_SetMode(GPIO_Desc[ulPin].P, GPIO_Desc[ulPin].bit, GPIO_PMD_INPUT);
        break ;

        case INPUT_PULLUP:            
            GPIO_SetMode(GPIO_Desc[ulPin].P, GPIO_Desc[ulPin].bit, GPIO_PMD_INPUT);
        break ;

        case OUTPUT:
        	GPIO_SetMode(GPIO_Desc[ulPin].P, GPIO_Desc[ulPin].bit, GPIO_PMD_OUTPUT); 
        break ;

        default:
        break ;
    }
}

extern void digitalWrite( uint32_t ulPin, uint32_t ulVal )
{
	#ifdef USE_BoardToPin
	if(ulPin > BoardToPin_MAX_COUNT) return;
	#endif
	
	if(GPIO_Desc[ulPin].P == NULL) return;	

	if(ulVal==HIGH)
		(GPIO_Desc[ulPin].P)->DOUT |= GPIO_Desc[ulPin].bit;	
	else
		(GPIO_Desc[ulPin].P)->DOUT &= ~GPIO_Desc[ulPin].bit;
}

extern int digitalRead( uint32_t ulPin )
{
	#ifdef USE_BoardToPin
	if(ulPin > BoardToPin_MAX_COUNT) return;     
	#endif
	if(GPIO_Desc[ulPin].P == NULL) return 0;
	
	return ((GPIO_Desc[ulPin].P)->PIN & GPIO_Desc[ulPin].bit)?HIGH:LOW;
}

#ifdef __cplusplus
}
#endif


