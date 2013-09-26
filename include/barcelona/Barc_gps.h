/* Barc_gps.h - header file of GPS device driver for Barcelona */

/* Copyright Mistral Software Pvt. Ltd. */

/*
modification history
--------------------
01a, 2oct03, bd created
*/

#ifndef __INCBarc_gpsh 
#define __INCBarc_gpsh 

#include <linux/ioctl.h>

#ifndef __INCBarc_Typesh
#include <barcelona/Barc_Types.h>
#endif /* __INCBarc_Typesh */

#ifdef __cplusplus 
extern "C" { 
#endif /* __cplusplus */ 

#define GPS_DEVNAME         "gps"


/* GPS major number */ 

#define BARCELONA_GPS_MAJOR_NUMBER      243

/* GPS driver magic number */ 

#define GPS_DRIVER_MAGIC    'U'

/* ioctl commands */

#define IOW_GPS_ON      _IO( GPS_DRIVER_MAGIC, 1 )
#define IOW_GPS_OFF     _IO( GPS_DRIVER_MAGIC, 2 )
#define IOW_GPS_RESET     _IO( GPS_DRIVER_MAGIC, 3 )
#define IOW_GPS_UPDATE_ON     _IO( GPS_DRIVER_MAGIC, 4 )
#define IOW_GPS_UPDATE_OFF     _IO( GPS_DRIVER_MAGIC, 5 )
#define IOR_GET_TIME_STAMP    _IO( GPS_DRIVER_MAGIC, 6 )

#ifdef __cplusplus
} 
#endif /* __cplusplus */
#endif /* __INCBarc_gpsh */
