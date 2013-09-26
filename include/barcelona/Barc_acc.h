/* barc_acc.h - acceleroemte header file */ 

/* Copyright Mistral Software Pvt. Ltd.*/ 

/* 

modification history 
-------------------- 
01a,13Oct03, SJ  written. 
02a,28Oct03, SJ 1)changed the major no for testing 
                  old value was 128,new value is 120 (line 28)
                2)struct timer_list pollTimer commented due to warnings (line 48)

*/ 

#ifndef __INCbarc_acch
#define __INCbarc_acch

#include <linux/ioctl.h>

#ifndef __INCBarc_Typesh
#include <barcelona/Barc_Types.h>
#endif /* __INCBarc_Typesh */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define     ACC_DEVNAME         "acc"

/* sampling rate ranges from 100hz to 1hz*/

#define     MAX_SAMPLERATE      100        
#define     MIN_SAMPLERATE      1

/* Global FIFO of size 256 ACCMETER_DATA */

#define     MAX_FIFO_SIZE  255
#define     ACC_DRIVER_MAGIC    'A'

/* ioctl commands */

#define     IOR_FIFOFILLED_SIZE     _IO(ACC_DRIVER_MAGIC, 0)
#define     IOW_ACC_SAMPLINGRATE    _IO(ACC_DRIVER_MAGIC, 1)
#define     IOW_WD_SET_TIME         _IO(ACC_DRIVER_MAGIC, 2)
#define     IOW_WD_KICK             _IO(ACC_DRIVER_MAGIC, 3)

/* Typedefs */

typedef struct s3c2410AccFIFO
    {
	UINT32 u32xData;       /* x axis data */
	UINT32 u32yData;       /* y axis data */
	UINT32 u32sTimeStamp;
	UINT32 u32usTimeStamp;
	//struct timeval timestamp;
	UINT32 u32Temperature; /* temperature */
	}ACCMETER_DATA;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCbarc_acch */

