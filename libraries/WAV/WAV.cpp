#include "WAV.h"
#include "../Wire/Wire.h"
#include "WM8978.h"
#include <../SPI/SPI.h>
#include <../SD/src/SD.h>

void PDMA_Open(uint32_t u32Mask)
{
    PDMA_GCR->GCRCSR |= (u32Mask << 8);
}

void PDMA_Close(void)
{
    PDMA_GCR->GCRCSR = 0;
}


void PDMA_SetTransferCnt(uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount)
{
    PDMA_T *pdma;
    pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));
    pdma->CSR = (pdma->CSR & ~PDMA_CSR_APB_TWS_Msk) | u32Width;
    switch(u32Width)
    {
        case PDMA_WIDTH_32:
            pdma->BCR = (u32TransCount << 2);
            break;

        case PDMA_WIDTH_8:
            pdma->BCR = u32TransCount;
            break;

        case PDMA_WIDTH_16:
            pdma->BCR = (u32TransCount << 1);
            break;

        default:
            ;
    }
}


void PDMA_SetTransferAddr(uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl)
{
    PDMA_T *pdma;
    pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    pdma->SAR = u32SrcAddr;
    pdma->DAR = u32DstAddr;
    pdma->CSR = (pdma->CSR & ~(PDMA_CSR_SAD_SEL_Msk | PDMA_CSR_DAD_SEL_Msk)) | (u32SrcCtrl | u32DstCtrl);
}

void PDMA_SetTransferMode(uint32_t u32Ch, uint32_t u32Peripheral, uint32_t u32ScatterEn, uint32_t u32DescAddr)
{
    uint32_t u32Index = 0;
    PDMA_T *pdma;
    pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    if(u32Peripheral > PDMA_PWM3_RX) /* Memory-to-Memory */
        pdma->CSR = (pdma->CSR & ~(PDMA_CSR_MODE_SEL_Msk));
    else if(u32Peripheral > PDMA_I2S_TX) /* Peripheral-to-Memory */
        pdma->CSR = (pdma->CSR & ~(PDMA_CSR_MODE_SEL_Msk) | (0x1 << PDMA_CSR_MODE_SEL_Pos));
    else /* Memory-to-Peripheral */
        pdma->CSR = (pdma->CSR & ~(PDMA_CSR_MODE_SEL_Msk) | (0x2 << PDMA_CSR_MODE_SEL_Pos));

    switch(u32Peripheral)
    {
        case 0:
            PDMA_GCR->PDSSR0 = (PDMA_GCR->PDSSR0 & ~PDMA_PDSSR0_SPI0_TXSEL_Msk) | (u32Ch << PDMA_PDSSR0_SPI0_TXSEL_Pos);
            break;
        case 1:
            PDMA_GCR->PDSSR0 = (PDMA_GCR->PDSSR0 & ~PDMA_PDSSR0_SPI1_TXSEL_Msk) | (u32Ch << PDMA_PDSSR0_SPI1_TXSEL_Pos);
            break;
        case 2:
            PDMA_GCR->PDSSR0 = (PDMA_GCR->PDSSR0 & ~PDMA_PDSSR0_SPI2_TXSEL_Msk) | (u32Ch << PDMA_PDSSR0_SPI2_TXSEL_Pos);
            break;
        case 3:
            PDMA_GCR->PDSSR1 = (PDMA_GCR->PDSSR1 & ~PDMA_PDSSR1_UART0_TXSEL_Msk) | (u32Ch << PDMA_PDSSR1_UART0_TXSEL_Pos);
            break;
        case 4:
            PDMA_GCR->PDSSR1 = (PDMA_GCR->PDSSR1 & ~PDMA_PDSSR1_UART1_TXSEL_Msk) | (u32Ch << PDMA_PDSSR1_UART1_TXSEL_Pos);
            break;
        case 5:
            PDMA_GCR->PDSSR2 = (PDMA_GCR->PDSSR2 & ~PDMA_PDSSR2_I2S_TXSEL_Msk) | (u32Ch << PDMA_PDSSR2_I2S_TXSEL_Pos);
            break;
        case 6:
            PDMA_GCR->PDSSR0 = (PDMA_GCR->PDSSR0 & ~PDMA_PDSSR0_SPI0_RXSEL_Msk) | (u32Ch << PDMA_PDSSR0_SPI0_RXSEL_Pos);
            break;
        case 7:
            PDMA_GCR->PDSSR0 = (PDMA_GCR->PDSSR0 & ~PDMA_PDSSR0_SPI1_RXSEL_Msk) | (u32Ch << PDMA_PDSSR0_SPI1_RXSEL_Pos);
            break;
        case 8:
            PDMA_GCR->PDSSR0 = (PDMA_GCR->PDSSR0 & ~PDMA_PDSSR0_SPI2_RXSEL_Msk) | (u32Ch << PDMA_PDSSR0_SPI2_RXSEL_Pos);
            break;
        case 9:
            PDMA_GCR->PDSSR1 = (PDMA_GCR->PDSSR1 & ~PDMA_PDSSR1_UART0_RXSEL_Msk) | (u32Ch << PDMA_PDSSR1_UART0_RXSEL_Pos);
            break;
        case 10:
            PDMA_GCR->PDSSR1 = (PDMA_GCR->PDSSR1 & ~PDMA_PDSSR1_UART1_RXSEL_Msk) | (u32Ch << PDMA_PDSSR1_UART1_RXSEL_Pos);
            break;
        case 11:
            PDMA_GCR->PDSSR2 = (PDMA_GCR->PDSSR2 & ~PDMA_PDSSR2_I2S_RXSEL_Msk) | (u32Ch << PDMA_PDSSR2_I2S_RXSEL_Pos);
            break;
        case 12:
            PDMA_GCR->PDSSR1 = (PDMA_GCR->PDSSR1 & ~PDMA_PDSSR1_ADC_RXSEL_Msk) | (u32Ch << PDMA_PDSSR1_ADC_RXSEL_Pos);
            break;
        case 13:
            PDMA_GCR->PDSSR2 = (PDMA_GCR->PDSSR2 & ~PDMA_PDSSR2_PWM0_RXSEL_Msk) | (u32Ch << PDMA_PDSSR2_PWM0_RXSEL_Pos);
            break;
        case 14:
            PDMA_GCR->PDSSR2 = (PDMA_GCR->PDSSR2 & ~PDMA_PDSSR2_PWM1_RXSEL_Msk) | (u32Ch << PDMA_PDSSR2_PWM1_RXSEL_Pos);
            break;
        case 15:
            PDMA_GCR->PDSSR2 = (PDMA_GCR->PDSSR2 & ~PDMA_PDSSR2_PWM2_RXSEL_Msk) | (u32Ch << PDMA_PDSSR2_PWM2_RXSEL_Pos);
            break;
        case 16:
            PDMA_GCR->PDSSR2 = (PDMA_GCR->PDSSR2 & ~PDMA_PDSSR2_PWM3_RXSEL_Msk) | (u32Ch << PDMA_PDSSR2_PWM3_RXSEL_Pos);
            break;

        default:/* select PDMA channel as memory to memory */
            for(u32Index = 0; u32Index < 8; u32Index++)
            {
                if((PDMA_GCR->PDSSR0 & (0xF << (u32Index * 4))) == (u32Ch << (u32Index * 4)))
                    PDMA_GCR->PDSSR0 |= 0xF << (u32Index * 4);
                if((PDMA_GCR->PDSSR1 & (0xF << (u32Index * 4))) == (u32Ch << (u32Index * 4)))
                    PDMA_GCR->PDSSR1 |= 0xF << (u32Index * 4);
                if((PDMA_GCR->PDSSR2 & (0xF << (u32Index * 4))) == (u32Ch << (u32Index * 4)))
                    PDMA_GCR->PDSSR2 |= 0xF << (u32Index * 4);
            }
    }
}

void PDMA_Trigger(uint32_t u32Ch)
{
    PDMA_T *pdma;
    pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    pdma->CSR |= (PDMA_CSR_TRIG_EN_Msk | PDMA_CSR_PDMACEN_Msk);
}

void PDMA_EnableInt(uint32_t u32Ch, uint32_t u32Mask)
{
    PDMA_T *pdma;
    pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    pdma->IER |= u32Mask;
}

void PDMA_DisableInt(uint32_t u32Ch, uint32_t u32Mask)
{
    PDMA_T *pdma;
    pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    pdma->IER &= ~u32Mask;
}


static uint32_t I2S_GetSourceClockFreq(I2S_T *i2s)
{
    uint32_t u32Freq, u32ClkSrcSel;

    u32ClkSrcSel = CLK->CLKSEL2 & CLK_CLKSEL2_I2S_S_Msk;

    switch(u32ClkSrcSel)
    {
        case CLK_CLKSEL2_I2S_S_HXT:
            u32Freq = __HXT;
            break;

        case CLK_CLKSEL2_I2S_S_PLL:
            u32Freq = CLK_GetPLLClockFreq();
            break;

        case CLK_CLKSEL2_I2S_S_HIRC:
            u32Freq = __HIRC;
            break;

        case CLK_CLKSEL2_I2S_S_HCLK:
            u32Freq = SystemCoreClock;
            break;

        default:
            u32Freq = __HIRC;
            break;
    }

    return u32Freq;
}

uint32_t I2S_Open(I2S_T *i2s, uint32_t u32MasterSlave, uint32_t u32SampleRate, uint32_t u32WordWidth, uint32_t u32Channels, uint32_t u32DataFormat)
{
    uint32_t u32Divider;
    uint32_t u32BitRate, u32SrcClk;

    /* Reset I2S */
    SYS->IPRSTC2 |= SYS_IPRSTC2_I2S_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2S_RST_Msk;

    /* Configure I2S controller according to input parameters. */
    i2s->CON = u32MasterSlave | u32WordWidth | u32Channels | u32DataFormat | I2S_FIFO_TX_LEVEL_WORD_4 | I2S_FIFO_RX_LEVEL_WORD_4;

    /* Get I2S source clock frequency */
    u32SrcClk = I2S_GetSourceClockFreq(i2s);

    /* Calculate bit clock rate */
    u32BitRate = u32SampleRate * (((u32WordWidth >> 4) & 0x3) + 1) * 16;
    u32Divider = ((u32SrcClk / u32BitRate) >> 1) - 1;
    i2s->CLKDIV = (i2s->CLKDIV & ~I2S_CLKDIV_BCLK_DIV_Msk) | (u32Divider << 8);

    /* Calculate real sample rate */
    u32BitRate = u32SrcClk / ((u32Divider + 1) * 2);
    u32SampleRate = u32BitRate / ((((u32WordWidth >> 4) & 0x3) + 1) * 16);

    /* Enable TX, RX and I2S controller */
    i2s->CON |= (I2S_CON_RXEN_Msk | I2S_CON_TXEN_Msk | I2S_CON_I2SEN_Msk);

    return u32SampleRate;
}

void I2S_Close(I2S_T *i2s)
{
    i2s->CON &= ~I2S_CON_I2SEN_Msk;
}

void I2S_EnableInt(I2S_T *i2s, uint32_t u32Mask)
{
    i2s->IE |= u32Mask;
}

void I2S_DisableInt(I2S_T *i2s, uint32_t u32Mask)
{
    i2s->IE &= ~u32Mask;
}

uint32_t I2S_EnableMCLK(I2S_T *i2s, uint32_t u32BusClock)
{
    uint32_t u32Divider;
    uint32_t u32SrcClk, u32Reg;

    u32SrcClk = I2S_GetSourceClockFreq(i2s);
    if(u32BusClock == u32SrcClk)
        u32Divider = 0;
    else
        u32Divider = (u32SrcClk / u32BusClock) >> 1;

    i2s->CLKDIV = (i2s->CLKDIV & ~I2S_CLKDIV_MCLK_DIV_Msk) | u32Divider;

    i2s->CON |= I2S_CON_MCLKEN_Msk;

    u32Reg = i2s->CLKDIV & I2S_CLKDIV_MCLK_DIV_Msk;

    if(u32Reg == 0)
        return u32SrcClk;
    else
        return ((u32SrcClk >> 1) / u32Reg);
}

void I2S_DisableMCLK(I2S_T *i2s)
{
    i2s->CON &= ~I2S_CON_MCLKEN_Msk;
}

WAV::WAV()
{

}

WAV::~WAV()
{
}

void WAV::parseFile(const char *name)
{
	u32 br=0;
	u8 buf[512]; 
	ChunkRIFF *riff;
	ChunkFMT *fmt;
	ChunkFACT *fact;
	ChunkDATA *data;
	
	wavFile = SD.open(name);
	if (wavFile) {
		wavFile.read(buf,512);
		riff = (ChunkRIFF *)buf;
		if(riff->Format == 0X45564157){
			fmt=(ChunkFMT *)(buf+12);
			fact=(ChunkFACT *)(buf+12+8+fmt->ChunkSize);
			
			if(fact->ChunkID == 0X74636166 || fact->ChunkID == 0X5453494C){
				wavctrl.datastart = 12+8+fmt->ChunkSize+8+fact->ChunkSize;
			}else wavctrl.datastart = 12+8+fmt->ChunkSize; {
				data = (ChunkDATA *)(buf+wavctrl.datastart);
			}
			if(data->ChunkID == 0X61746164){
				wavctrl.audioformat=fmt->AudioFormat;
				wavctrl.nchannels=fmt->NumOfChannels;
				wavctrl.samplerate=fmt->SampleRate;
				wavctrl.bitrate=fmt->ByteRate*8;
				wavctrl.blockalign=fmt->BlockAlign;	
				wavctrl.bps=fmt->BitsPerSample;	
				
				wavctrl.datasize=data->ChunkSize;
				wavctrl.datastart=wavctrl.datastart+8;	
				#if (WAV_DBG == 1)
				Serial.print("wavctrl->audioformat:");Serial.println(wavctrl.audioformat);
				Serial.print("wavctrl->nchannels:");Serial.println(wavctrl.nchannels);
				Serial.print("wavctrl->samplerate:");Serial.println(wavctrl.samplerate);
				Serial.print("wavctrl->bitrate:");Serial.println(wavctrl.bitrate);
				Serial.print("wavctrl->blockalign:");Serial.println(wavctrl.blockalign);
				Serial.print("wavctrl->bps:");Serial.println(wavctrl.bps);
				Serial.print("wavctrl->datasize:");Serial.println(wavctrl.datasize);
				Serial.print("wavctrl->datastart:");  Serial.println(wavctrl.datastart);
				#endif
			}
		}
	}else{
		Serial.println("open file failed!!!");
	}
	
}
void WAV::begin(const char *name)
{
	u32 nchannels;
	parseFile(name);
	nchannels = I2S_MONO;
	if(wavctrl.nchannels ==2)
		nchannels = I2S_STEREO;
	Wire.begin();

	SYS_UnlockReg();
	CLK_EnableModuleClock(I2S_MODULE);
	/* Set GPA multi-function pins for I2S MCLK. */
	SYS->GPA_MFP |= SYS_GPA_MFP_PA15_I2S_MCLK;
	/* Set multi function pin for I2S: GPC0, GPC1, GPC2, GPC3, GPA15 */
	SYS->GPC_MFP |= SYS_GPC_MFP_PC0_I2S_LRCLK | SYS_GPC_MFP_PC1_I2S_BCLK | SYS_GPC_MFP_PC2_I2S_DI | SYS_GPC_MFP_PC3_I2S_DO;
	SYS->ALT_MFP |= SYS_ALT_MFP_PA15_I2S_MCLK | SYS_ALT_MFP_PC0_I2S_LRCLK | SYS_ALT_MFP_PC1_I2S_BCLK | SYS_ALT_MFP_PC2_I2S_DI | SYS_ALT_MFP_PC3_I2S_DO;

	//CLK_SetModuleClock(I2S_MODULE, CLK_CLKSEL2_I2S_S_HXT, 0); //12M HZ
	if(wavctrl.samplerate == 44100){
		CLK_SetModuleClock(I2S_MODULE, CLK_CLKSEL2_I2S_S_PLL, 0); //44117Hz
	}else if(wavctrl.samplerate == 32000){
		CLK_SetModuleClock(I2S_MODULE, CLK_CLKSEL2_I2S_S_HCLK, 0); //32142Hz
	}else if(wavctrl.samplerate == 24000){
		CLK_SetModuleClock(I2S_MODULE, CLK_CLKSEL2_I2S_S_PLL, 0); //24193Hz
	}else if(wavctrl.samplerate == 16000){
		CLK_SetModuleClock(I2S_MODULE, CLK_CLKSEL2_I2S_S_HIRC, 0); //15709Hz
		wavctrl.samplerate = 15709;
	}else if(wavctrl.samplerate == 8000){
		//CLK_SetModuleClock(I2S_MODULE, CLK_CLKSEL2_I2S_S_HCLK, 0); //8035Hz
		CLK_SetModuleClock(I2S_MODULE, CLK_CLKSEL2_I2S_S_HCLK, 0); //8035Hz
		//CLK_SetModuleClock(I2S_MODULE, CLK_CLKSEL2_I2S_S_HIRC, 0); //8037Hz
	}else{
		CLK_SetModuleClock(I2S_MODULE, CLK_CLKSEL2_I2S_S_HXT, 0); //12M HZ
	}
	{
		u32 SrcClk = I2S_GetSourceClockFreq(I2S);
		#if (WAV_DBG == 1)
		Serial.print("IIS CLK=");Serial.println(SrcClk);
		#endif
	}
	SYS_LockReg();
	WM8978_Init();
	WM8978_HPvol_Set(lVolume,rVolume);
	WM8978_SPKvol_Set(0);


	WM8978_ADDA_Cfg(1,0);
	WM8978_Input_Cfg(0,0,0);
	WM8978_Output_Cfg(1,0);

	WM8978_I2S_Cfg(2,0);
	
	u32 freq=I2S_Open(I2S, I2S_MODE_MASTER, wavctrl.samplerate, I2S_DATABIT_16, nchannels, I2S_FORMAT_I2S);
	I2S_EnableMCLK(I2S, 256*wavctrl.samplerate);
	//Serial.print("IIS clk=");Serial.println(freq);
	//Serial.println("WAV::begin");
}

int WAV::play(const char *name)
{
    begin(name);
	#if (WAV_DBG == 1)
	Serial.println("WAV::play");
	#endif
	if(	!wavFile ) {
		#if (WAV_DBG == 1)
		Serial.println("WAV::play-->wavFile Closed");
		#endif
		return WAV_DECODE_END;
	}
	wavFile.seek(wavctrl.datastart);
	//read some frames
	//Serial.print("HCLK=");Serial.println(CLK_GetHCLKFreq());
	//Serial.print("PLL =");Serial.println(CLK_GetPLLClockFreq());
	//Serial.print("HXT =");Serial.println(CLK_GetHXTFreq());
	//Serial.print("CPU =");Serial.println(CLK_GetCPUFreq());

	for(int i=0;i<WAV_BUF_SIZE;i++)
		decode();
	isRecoder = FALSE;
	SYS_UnlockReg();
	CLK_EnableModuleClock(PDMA_MODULE);
	CLK->AHBCLK |= CLK_AHBCLK_PDMA_EN_Msk;
	I2S_ENABLE_TXDMA(I2S);
	SYS_LockReg();

	PDMA_Open(1 << 0);

	triggerDMA(0);
	NVIC_SetPriority(PDMA_IRQn, 1);
	PDMA_EnableInt(0, PDMA_IER_BLKD_IE_Msk);
	NVIC_EnableIRQ(PDMA_IRQn);
	status = WAV_PLAY;
	return WAV_DECODING;
}

//
//0    ,1     2     3     4     5
//rI
//wI

//wI    rI

//rI                wI
//            rI    wI
int WAV::decode(void){
	static char first = 1;
	if(!wavFile) { 
		return status = WAV_DECODE_END;
	}
	if(((readIndex+1)%WAV_BUF_SIZE) == writeIndex){ //full
		//Serial.println("full");
		return 0;
	}
	
	if(wavFile.read(&wavBuf[readIndex][0],WAV_BUF_LINE_BYTE) != WAV_BUF_LINE_BYTE){
		wavFile.close();
		status = WAV_CLOSED;
		stop();
		#if (WAV_DBG == 1)
		Serial.println("stop");
		#endif
	}else{
		availableSize += WAV_BUF_LINE_BYTE;
	}
	if(first){
		#if (WAV_DBG == 1)
		for(int i=0; i<4; i++){
			Serial.print(wavBuf[readIndex][i],HEX);Serial.print(' ');
		}
		Serial.println();
		#endif
		first = 0;
	}
	readIndex = ( readIndex + 1 ) % WAV_BUF_SIZE;
	return status = WAV_DECODING;
}

int WAV::pause(void)
{
	status = WAV_PAUSE;
	NVIC_DisableIRQ(PDMA_IRQn);
	PDMA_DisableInt(0, PDMA_IER_BLKD_IE_Msk);
	return 0;
}

int WAV::continus(void)
{
	//enable_iis_irq();
	status = WAV_PLAY;
	PDMA_EnableInt(0, PDMA_IER_BLKD_IE_Msk);
	NVIC_EnableIRQ(PDMA_IRQn);
	return 0;
}

int WAV::stop(void)
{
	NVIC_DisableIRQ(PDMA_IRQn);
	PDMA_DisableInt(0, PDMA_IER_BLKD_IE_Msk);
	status = WAV_STOP;
	if(isRecoder){
		__WaveHeader *wavhead = (__WaveHeader *)malloc(sizeof(__WaveHeader));
		if(wavFile && wavhead){
			wavFile.seek(0);
			wavFile.read(wavhead, sizeof(__WaveHeader));
			wavhead->riff.ChunkSize = wavsize+36;
			wavhead->data.ChunkSize = wavsize;
			wavFile.seek(0);
			wavFile.write((const char *)wavhead, sizeof(__WaveHeader));
			wavFile.close();
		}
		if(wavhead)
			free(wavhead);
	}
	return 0;
}

int WAV::record(const char *name)
{
	wavFile = SD.open(name, FILE_WRITE | O_TRUNC);
	if(!wavFile){
		#if (WAV_DBG == 1)
		Serial.println("create file failed!!!");
		#endif
		return -1;
	}
	__WaveHeader wavhead;
	
	//riff block
	wavhead.riff={
		0X46464952,//u32 ChunkID;			///< chunk id;"RIFF", 0X46464952
		0,//u32 ChunkSize ; 		///< filesize-8
		0X45564157//u32 Format; 			///< WAVE,0X45564157

	};
	
	//fmt block
	wavhead.fmt={
		0x20746D66,//u32 ChunkID;			///< chunk id: "fmt " equal 0X20746D66
		0x0010, //u32 ChunkSize ; 		///< (exclude ID and Size); it is 16 here.
		0x0001, //u16 AudioFormat;		///< 0X01,line PCM; 0X11 IMA ADPCM
		0x0001, //u16 NumOfChannels;		///<  2 strevo	 1 mono
		0x00002EE0,//0x00005DC0,//u32 SampleRate; 		///< ex. 0X5DC0 = 24000hz
		0x00005DC0,//0x0000BB80,//u32 ByteRate;			 
		0x0002,//	u16 BlockAlign; 		//
		0x0010 //	u16 BitsPerSample;		//2bytes = 16bits
		//0x0000//	u16 ByteExtraData;		//no this parameter for line pcm
	};
	
	//data block
	wavhead.data={
		0x61746164, //u32 ChunkID;		   	//chunk id: "data" equal 0X61746164
		0x00000000  //u32 ChunkSize ;		   	//(exclude ID and Size) 
	};
	wavFile.write((const char *)&wavhead,sizeof(wavhead));
	{
			wavsize = 512-sizeof(wavhead);
			char *p = (char *)malloc(wavsize);
			if(p){
				memset(p,0,wavsize);
				wavFile.write((const char *)p,wavsize);
				free(p);
			}else{
				wavsize = 0;
			}
	}
	//setup  IIS
	Wire.begin();

	SYS_UnlockReg();
	CLK_EnableModuleClock(I2S_MODULE);
	/* Set GPA multi-function pins for I2S MCLK. */
	SYS->GPA_MFP |= SYS_GPA_MFP_PA15_I2S_MCLK;
	/* Set multi function pin for I2S: GPC0, GPC1, GPC2, GPC3, GPA15 */
	SYS->GPC_MFP |= SYS_GPC_MFP_PC0_I2S_LRCLK | SYS_GPC_MFP_PC1_I2S_BCLK | SYS_GPC_MFP_PC2_I2S_DI | SYS_GPC_MFP_PC3_I2S_DO;
	SYS->ALT_MFP |= SYS_ALT_MFP_PA15_I2S_MCLK | SYS_ALT_MFP_PC0_I2S_LRCLK | SYS_ALT_MFP_PC1_I2S_BCLK | SYS_ALT_MFP_PC2_I2S_DI | SYS_ALT_MFP_PC3_I2S_DO;
	CLK_SetModuleClock(I2S_MODULE, CLK_CLKSEL2_I2S_S_PLL, 0); //24193Hz
	SYS_LockReg();

	u32 SrcClk = I2S_GetSourceClockFreq(I2S);
	Serial.print("IIS CLK=");Serial.println(SrcClk);
	
	WM8978_Init();
	WM8978_HPvol_Set(0,0);
	WM8978_SPKvol_Set(0);
	WM8978_ADDA_Cfg(0,1);
	//WM8978_Input_Cfg(1,0,0);
	WM8978_Output_Cfg(0,1);
	WM8978_Noise_Set(1,0);
	WM8978_ALC_Set(1, 7, 7);
	if(recordSource == RECORD_SOURCE_MIC){
		WM8978_Input_Cfg(1,0,0);
		WM8978_MIC_Gain(63);
	}else if(recordSource == RECORD_SOURCE_LINEIN){
		WM8978_Input_Cfg(0,1,0);
		WM8978_LINEIN_Gain(7);
	}
	WM8978_I2S_Cfg(2,0);

	u32 freq=I2S_Open(I2S, I2S_MODE_MASTER, 12000, I2S_DATABIT_16, I2S_MONO, I2S_FORMAT_I2S);
	I2S_EnableMCLK(I2S, 256*12000);
	#if (WAV_DBG == 1)
	Serial.print("IIS clk=");Serial.println(freq);
	Serial.println("WAV::record begin");
	#endif
	isRecoder = TRUE;

	SYS_UnlockReg();
	CLK_EnableModuleClock(PDMA_MODULE);
	CLK->AHBCLK |= CLK_AHBCLK_PDMA_EN_Msk;
	I2S_ENABLE_RXDMA(I2S);
	SYS_LockReg();


	PDMA_Open(1 << 0);
	triggerDMA(1);
	NVIC_SetPriority(PDMA_IRQn, 2);
	PDMA_EnableInt(0, PDMA_IER_BLKD_IE_Msk);
	NVIC_EnableIRQ(PDMA_IRQn);

	return 0;
}

//
//0    ,1     2     3     4     5
//rI
//wI

//wI    rI

//rI                wI
//            rI    wI

int WAV::encode(void)
{
	static int counter = 0;
	if(!wavFile) {
		Serial.println("file error");
		return WAV_ENCODE_END;
	}
	if(writeIndex == readIndex){ //empty
		return WAV_ENCODING;
	}
	
	if(wavFile.write((const uint8_t *)&wavBuf[writeIndex][0],WAV_BUF_LINE_BYTE) != WAV_BUF_LINE_BYTE){
		stop();
		wavFile.close();
		status = WAV_CLOSED;
		#if (WAV_DBG == 1)
		Serial.println("stop");
		#endif
	}else{
		wavsize += WAV_BUF_LINE_BYTE;
	}
	{
		#if (WAV_DBG == 1)
		++counter;
		Serial.println(counter);
		#endif
	}
	writeIndex = (writeIndex+1)%WAV_BUF_SIZE;
	return WAV_ENCODING;
}

int WAV::setVolume(int lVolume_,int rVolume_)
{
	if(lVolume_ < 0) lVolume_ = 0;
	else if(lVolume_ > 63) lVolume_ = 63;
	
	if(rVolume_ < 0) rVolume_ = 0;
	else if(rVolume_ > 63) rVolume_ = 63;

	lVolume = lVolume_;
	rVolume = rVolume_;
	
	if(status != WAV_DEFAULT){
		WM8978_HPvol_Set(lVolume,rVolume);
	}
	return 0;
}

void WAV::getVolume(int &lVolume_,int &rVolume_)
{
	lVolume_ = lVolume;
	rVolume_ = rVolume;
}

void WAV::setRecoderSource(int recordSource_)
{
	if(recordSource_ == RECORD_SOURCE_MIC || recordSource_ == RECORD_SOURCE_LINEIN)
		recordSource = recordSource_;
}

char WAV::name_[13];
char WAV::status = WAV_DEFAULT;
unsigned int WAV::wavsize=0;

__wavctrl WAV::wavctrl;
File WAV::wavFile;
u32 WAV::wavBuf[WAV_BUF_SIZE][WAV_BUF_LINE];
u16 WAV::wavlength[2]={0};
volatile int WAV::readIndex = 0;
volatile int WAV::writeIndex = 0;
volatile u32 WAV::availableSize=0;
boolean WAV::isRecoder;
int WAV::recordSource = RECORD_SOURCE_MIC;
int WAV::lVolume = 45;
int WAV::rVolume = 45;

WAV wav;

void triggerDMA(int inout)
{
	if(inout == 0){
		unsigned int *p = &WAV::wavBuf[WAV::writeIndex][0];
	    PDMA_SetTransferCnt(0, PDMA_WIDTH_32, WAV_BUF_LINE);
	    PDMA_SetTransferAddr(0, (u32)p, PDMA_SAR_INC, (u32)(&((I2S)->TXFIFO)), PDMA_DAR_FIX);
		PDMA_SetTransferMode(0, PDMA_I2S_TX, FALSE, 0);
	    PDMA_Trigger(0);
		WAV::writeIndex = (WAV::writeIndex+1)%WAV_BUF_SIZE;
		WAV::availableSize -= WAV_BUF_LINE_BYTE;
	}else{
		unsigned int *p = &WAV::wavBuf[WAV::readIndex][0];
	    PDMA_SetTransferCnt(0, PDMA_WIDTH_32, WAV_BUF_LINE);
	    PDMA_SetTransferAddr(0, (u32)(&((I2S)->RXFIFO)), PDMA_SAR_FIX, (u32)p, PDMA_DAR_INC);
		PDMA_SetTransferMode(0, PDMA_I2S_RX, FALSE, 0);
	    PDMA_Trigger(0);
		WAV::readIndex = (WAV::readIndex+1)%WAV_BUF_SIZE;
	}
}

void PDMA_IRQHandler(void)
{
	uint32_t status = PDMA_GET_INT_STATUS();

    if(status & 0x1){            /* CH0 */  
        PDMA_CLR_CH_INT_FLAG(0, PDMA_ISR_BLKD_IF_Msk);
    }else if(status & 0x2){      /* CH1 */
        PDMA_CLR_CH_INT_FLAG(1, PDMA_ISR_BLKD_IF_Msk);
    }else if(status & 0x4){      /* CH2 */
        PDMA_CLR_CH_INT_FLAG(2, PDMA_ISR_BLKD_IF_Msk);
    }else if(status & 0x8){      /* CH3 */
        PDMA_CLR_CH_INT_FLAG(3, PDMA_ISR_BLKD_IF_Msk);
    }else if(status & 0x10){      /* CH4 */
        PDMA_CLR_CH_INT_FLAG(4, PDMA_ISR_BLKD_IF_Msk);
    }else if(status & 0x20){      /* CH5 */
        PDMA_CLR_CH_INT_FLAG(5, PDMA_ISR_BLKD_IF_Msk);
    }
	triggerDMA(WAV::isRecoder);
}

