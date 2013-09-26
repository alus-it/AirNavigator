/* Barc_Battery.h - battery driver header file */ 
 
/* Copyright Mistral Software Pvt. Ltd.*/ 

/* 
modification history 
-------------------- 
01a,3oct03,rks  written. 
*/ 
 
#ifndef __INCBarc_Batteryh 
#define __INCBarc_Batteryh

#include <linux/ioctl.h>

#ifndef __INCBarc_Typesh
#include <barcelona/Barc_Types.h>
#endif /* __INCBarc_Typesh */

#ifdef __cplusplus 
extern "C" { 
#endif /* __cplusplus */ 

/* defines */

#define BATTERY_DEVNAME         "battery"

#define BATTERY_DRIVER_MAGIC    'B' /* Battery driver magic number */

#define IOR_BATTERY_STATUS      _IOR(BATTERY_DRIVER_MAGIC, 0, \
								        sizeof (BATTERY_STATUS))

#define IOW_BATTERY_CHARGER_ENABLE          _IO(BATTERY_DRIVER_MAGIC, 1)
#define IOW_BATTERY_CHARGER_DISABLE         _IO(BATTERY_DRIVER_MAGIC, 2)

#define POWER_OFF               0x00 /* device is powered off */
#define POWER_ON                0x01 /* device is powered on */

/* typedefs */

typedef struct batteryStatus
    {
    UINT16  u16BatteryVoltage;      /* battery voltage */
    UINT16  u16ReferenceVoltage;    /* reference voltage */
    UINT16  u16ChargeCurrent;       /* charge current */
    } BATTERY_STATUS;

#ifdef __cplusplus
} 
#endif /* __cplusplus */ 
 
#endif /* __INCBarc_Batteryh */
