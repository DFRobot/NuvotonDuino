#include "misc.h"

#define EEPROM_BASE (0x10C00)
#define EEPROM_SECOND_PAGE (EEPROM_BASE+512)
#define EEPROM_END (EEPROM_BASE+1024)
#define EEPROM_SIZE 1024

u8 eeprom_read_byte(u8 *addr)
{
	if(((u32)addr) >= EEPROM_SIZE){
		return 0;
	}
	//Serial.print("read addr=");Serial.println((u32)addr);
	return *((u8 *)((u32)addr+EEPROM_BASE));	
}

void eeprom_write_byte(u8* addr,u8 value)
{
	u8 * buf,*p;
	u32 Data;
	//Serial.print("addr = 0x");Serial.println((u32)addr,HEX);
	//Serial.print("value = ");Serial.println(value);

	if(((u32)addr) >= EEPROM_SIZE){
		return;
	}
	buf = malloc(512);
	if(!buf)
		return;
	p = buf;
	memcpy(buf,EEPROM_BASE+((u32)addr/512*512),512);
	buf[((u32)addr)%512] = value;
	SYS_UnlockReg();
	FMC->ISPCON |= 1;
 	FMC_EnableAPUpdate();
	FMC_Erase(EEPROM_BASE+((u32)addr/512*512));
	for(int count = 0; count < 512; count+=4){
		Data = p[0]+(p[1]<<8)+(p[2]<<16)+(p[3]<<24);
		//Serial.print(Data,HEX);Serial.print(' ');
		FMC_Write(EEPROM_BASE+((u32)addr/512*512)+count,Data);
		p += 4;
	}
	//Serial.println();
	FMC->ISPCON &= ~1;
  	FMC_DisableAPUpdate();
	SYS_LockReg();
	free(buf);
}


void hook(void)
{
	pinMode(RX_LED,OUTPUT);
	pinMode(TX_LED,OUTPUT);
	digitalWrite(RX_LED,HIGH);
	digitalWrite(TX_LED,HIGH);
	
	#if defined(__BlunoM0xxx__)
	UNLOCKREG();						//en: Unlock the locked registers
    FMC_ENABLE_ISP();                             //en:Enable ISP
    LOCKREG(); //en: Re-lock the locked registers
    UDC_FlashInit();
    #endif
}

