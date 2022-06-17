#ifndef _INTRFACE_H_
#define _INTRFACE_H_

#include "util.h"

/************************************************************************
* function definition
*************************************************************************/

/************************************************************************
* algorithm utility functions		
*************************************************************************/
int algoPreset(unsigned char *setAlgoPtr, unsigned int setAlgoSize);
int algoInit(void);
int algoGetByte(unsigned char *byteOut);
int algoFinal(void);

/************************************************************************
* data utility functions
*************************************************************************/
typedef struct toc{
	unsigned char ID;
	unsigned int uncomp_size;
	unsigned char compression;
	unsigned int address;
} DATA_TOC;

typedef struct dataPointer{
	unsigned char ID;
	unsigned int address;
} DATA_BUFFER;

int dataPreset(unsigned char *setDataPtr, unsigned int setDataSize);
int dataInit(void);			// initialize data
int dataGetByte(unsigned char *byteOut, short int incCurrentAddr, CSU *checksumUnit);	// get one byte from current column
int dataReset(unsigned char isResetBuffer);							// reset data pointer
int dataFinal(void);

int HLDataGetByte(unsigned char dataSet, unsigned char *dataByte, unsigned int uncomp_bitsize);
int dataReadthroughComment(void);
unsigned char getRequestNewData(void);
int dataLoadTOC(short int storeTOC);
int dataRequestSet(unsigned char dataSet);

/************************************************************************
* decompression utility functions
*************************************************************************/

void set_compression(unsigned char cmp);
unsigned char get_compression(void);
short int decomp_initFrame(int bitSize);
short int decomp_getByte(unsigned char *byteOut);
short int decomp_getNum(void);

#endif
