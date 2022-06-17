#ifndef _GRIF_IO_H_
#define _GRIF_IO_H_

#include <linux/gpio.h>

#define CSR_CNT		8192
#define CSRADDR(n)	(n * 4)
#define CSR_SIZE	4
#define CSR_SPACE	(CSR_CNT * CSR_SIZE)

#define REG_SIZE	2 /*FPGA registers is 16-bit wide*/
#define REG_OFFSET(n)	(REG_SIZE * n)

//#define IODEBUG
#ifdef IODEBUG
#define _IODEBUG 1
#else
#define _IODEBUG 0
#endif

#define NAME "grif_io"

#define iodbg(fmt, args...)				\
	do {						\
		if (_IODEBUG)				\
			pr_err(NAME": "fmt, ##args);	\
	} while (0)

#define EIM_BLOCK_SIZE	2

#endif //_GRIF_IO_H_
