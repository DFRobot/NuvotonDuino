#ifndef __WAV_H
#define __WAV_H
#include <Arduino.h>
#include "types.h"

///< RIFF block
typedef  struct
{
    u32 ChunkID;		   	///< chunk id;"RIFF", 0X46464952
    u32 ChunkSize ;		   	///< filesize-8
    u32 Format;	   			///< WAVE,0X45564157
}ChunkRIFF __attribute((aligned (1)));

//< fmt block
typedef  struct
{
    u32 ChunkID;		   	///< chunk id: "fmt " equal 0X20746D66
    u32 ChunkSize ;		   	///< (exclude ID and Size); it is 20 here.
    u16 AudioFormat;	  	///< 0X01,line PCM; 0X11 IMA ADPCM
	u16 NumOfChannels;		///<  2 strevo   1 mono
	u32 SampleRate;			///< ex. 0X1F40 = 8Khz
	u32 ByteRate;			 
	u16 BlockAlign;			//
	u16 BitsPerSample;		//
	//u16 ByteExtraData;		//no this parameter for line pcm
}ChunkFMT __attribute((aligned (1)));

///< fact block
typedef  struct 
{
    u32 ChunkID;		   	///< chunk id; "fact" equal 0X74636166;
    u32 ChunkSize ;		   	///< (exclule ID and Size); it is 4 here.
    u32 NumOfSamples;	  	
}ChunkFACT __attribute((aligned (1)));

//LIST block
typedef  struct 
{
    u32 ChunkID;		   	//chunk id: "LIST" equal 0X74636166;
    u32 ChunkSize ;		   	//(exclude ID and Size);it is 4 here. 
}ChunkLIST __attribute((aligned (1)));

//data block
typedef  struct 
{
    u32 ChunkID;		   	//chunk id: "data" equal 0X61746164
    u32 ChunkSize ;		   	//(exclude ID and Size) 
}ChunkDATA __attribute((aligned (1)));

//wav header
typedef  struct
{ 
	ChunkRIFF riff;	//riff block
	ChunkFMT fmt;  	//fmt block
//	ChunkFACT fact;	//fact block. no this block for line PCM	 
	ChunkDATA data;	//data block
}__WaveHeader __attribute((aligned (1))); 

//wav control struct
typedef  struct
{ 
    u16 audioformat;			//format;0X01,line PCM;  0X11 IMA ADPCM
	u16 nchannels;				// 1 indicate mono¡£2 indicate stereo
	u16 blockalign;				
	u32 datasize;				//WAV date size

    u32 totsec ;				//music total track. in terms of seconds
    u32 cursec ;				//currunt time. in terms of seconds
	
    u32 bitrate;	   			
	u32 samplerate;				 
	u16 bps;					//bits,for 16bit,24bit,32bit
	
	u32 datastart;				// data offset
}__wavctrl __attribute((aligned (1))); 

#define WAV_BUF_SIZE 2
#define WAV_BUF_LINE_BYTE (512)
#define WAV_BUF_LINE (WAV_BUF_LINE_BYTE/4)

#define RECORD_SOURCE_MIC    1
#define RECORD_SOURCE_LINEIN 2

class File;
class WAV
{
public:
	WAV();
	~WAV();
	void begin(const char *name);
	void parseFile(const char *name);
	int play(const char *name);
	int pause(void);
	int continus(void);
	int stop(void);
	int decode(void);
	int record(const char *name);
	int encode(void);
	int setVolume(int lVolume_,int rVolume_);
	void getVolume(int &lVolume_,int &rVolume_);
	void setRecoderSource(int recordSource_);
public:
	static File wavFile;
	static char name_[13];
	static char status;
	static unsigned int wavsize;
	static __wavctrl wavctrl;
	static u32 wavBuf[WAV_BUF_SIZE][WAV_BUF_LINE];
	static u16 wavlength[2];
	static volatile u32 availableSize;
	static volatile int readIndex,writeIndex;
	static boolean isRecoder;
	static int recordSource;
	static int lVolume,rVolume;
};

#define WAV_DBG 1

extern WAV wav;
void triggerDMA(int);

#define WAV_DEFAULT  0
#define WAV_PLAY     1
#define WAV_RECORD   2
#define WAV_PAUSE    3
#define WAV_CONTINUS 4
#define WAV_STOP     5
#define WAV_CLOSED   6

#define WAV_DECODING     0
#define WAV_DECODE_END   1

#define WAV_ENCODING     0
#define WAV_ENCODE_END   1

#endif
