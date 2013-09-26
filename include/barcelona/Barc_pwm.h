/* Barc_pwm.h - header file of PWM device driver for Barcelona */

/* Copyright Mistral Software Pvt. Ltd. */

/*
modification history
--------------------
01a, 2oct03, bd created
*/

#ifndef __INCBarc_pwmh 
#define __INCBarc_pwmh 

#include <linux/ioctl.h>

#ifndef __INCBarc_Typesh
#include <barcelona/Barc_Types.h>
#endif /* __INCBarc_Typesh */

#ifdef __cplusplus 
extern "C" { 
#endif /* __cplusplus */ 


#define PWM_DEVNAME         "pwm"

/* PWM major number */ 

#define BARCELONA_PWM_MAJOR_NUMBER 251

/* PWM driver majic number */ 

#define PWM_DRIVER_MAGIC    'T'

/* ioctl commands */

#define IOW_BACKLIGHT_INCREASE    _IO (PWM_DRIVER_MAGIC, 0)
#define IOW_BACKLIGHT_DECREASE    _IO (PWM_DRIVER_MAGIC, 1)
#define IOW_BACKLIGHT_OFF    _IO (PWM_DRIVER_MAGIC, 2)
#define IOW_BACKLIGHT_ON    _IO (PWM_DRIVER_MAGIC, 3)
#define IOW_CONTRAST_INCREASE    _IO (PWM_DRIVER_MAGIC, 4)
#define IOW_CONTRAST_DECREASE    _IO (PWM_DRIVER_MAGIC, 5)
#define IOW_LCD_ON    _IO (PWM_DRIVER_MAGIC, 6)
#define IOW_LCD_OFF    _IO (PWM_DRIVER_MAGIC, 7)
#define IOW_CONTRAST_UPDATE     _IO (PWM_DRIVER_MAGIC, 8) 
#define IOW_BACKLIGHT_UPDATE    _IO (PWM_DRIVER_MAGIC, 9) 
#define IOR_CONTRAST_CURRENT    _IO (PWM_DRIVER_MAGIC, 10) 
#define IOR_BACKLIGHT_CURRENT   _IO (PWM_DRIVER_MAGIC, 11) 

/* back light related macros */

#define MAX_BACK_LIGHT      19            
#define MIN_BACK_LIGHT      0            
#define DEFAULT_BACK_LIGHT  10 
    

/* contrast related macros */

#define DEFAULT_CONTRAST    3          
#define MIN_CONTRAST        0            
#define MAX_CONTRAST        8            
#define CONTRAST_BIT_WEIGHT2_DISPLACEMENT   ( 4 )
#define CONTRAST_BIT_WEIGHT1_DISPLACEMENT   ( 6 )
#define CONTRAST_BIT_WEIGHT0_DISPLACEMENT   ( 0 ) 

#ifdef __cplusplus
} 
#endif /* __cplusplus */ 
#endif /* __INCBarc_pwmh */
