#ifndef _HARDWARE_H_

#define _HARDWARE_H_

int lattice_spi_transmit(unsigned char *trBuffer, int trCount);
int lattice_spi_receive(unsigned char *rcBuffer, int rcCount);

int lattice_spi_pull_cs_high(void);
int lattice_spi_pull_cs_low(void);

/************************************************************************
* 
* Function Definition
* 
*************************************************************************/
#define DISPLAY			1
#define LOG_DISPLAY		1
/************************************************************************
* Hardware functions
*************************************************************************/
int SPI_init(void);
int SPI_final(void);
int wait(int ms);

/************************************************************************
* SPI transmission functions
*************************************************************************/
int TRANS_starttranx(unsigned char channel);
int TRANS_endtranx(void);
int TRANS_cstoggle(unsigned char channel);
int TRANS_runClk(void);
int TRANS_transmitBytes(unsigned char *trBuffer, int trCount);
int TRANS_receiveBytes(unsigned char *rcBuffer, int rcCount);

int TRANS_transceive_stream(int trCount, unsigned char *trBuffer, 
							int trCount2, int flag, unsigned char *trBuffer2,
							int mask_flag, unsigned char *maskBuffer);


/************************************************************************
* debug utility functions
*
* If you wish to enable debugging functionalities, uncomment definition
* DEBUG_LEVEL_1
*************************************************************************/
//#define	DEBUG_LEVEL_1	1
#ifdef	DEBUG_LEVEL_1
#define	DEBUG_LEVEL_2	1
#endif
#ifdef	DEBUG_LEVEL_2
#define	DEBUG_LEVEL_3	1
#endif

/************************************************************************
* debugging level
* for debug level 1, uncomment definition of DEBUG_LEVEL_1
* for debug level 2, uncomment definition of DEBUG_LEVEL_1. DEBUG_LEVEL_2
* for debug level 3, uncomment definition of DEBUG_LEVEL_1, DEBUG_LEVEL_2, DEBUG_LEVEL_3
*************************************************************************/

int dbgu_init(void);
void dbgu_putint(int debugCode, int debugCode2);

#ifdef	DEBUG_LEVEL_1
#include "debug.h"

#define	DEBUG_LEVEL_2	1
#endif

#ifdef	DEBUG_LEVEL_2
#define	DEBUG_LEVEL_3	1
#endif

#endif
