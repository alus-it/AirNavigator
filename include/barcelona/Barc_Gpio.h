/* Barc_Gpio.h - GPIO driver header file */ 
 
/* Copyright Mistral Software Pvt. Ltd.*/ 

/* 
modification history 
-------------------- 
01a,7oct03,rks  written. 
*/ 
 
#ifndef __INCBarc_Gpioh 
#define __INCBarc_Gpioh

#include <linux/ioctl.h>

#ifndef __INCBarc_Typesh
#include <barcelona/Barc_Types.h>
#endif /* __INCBarc_Typesh */

#ifdef __cplusplus 
extern "C" { 
#endif /* __cplusplus */ 


/* defines */

#define GPIO_DEVNAME       "hwstatus"

#define GPIO_DRIVER_MAGIC  'G' /* magic number for Gpio driver */

#define IOR_HWSTATUS                    _IOR(GPIO_DRIVER_MAGIC, 0, \
                                             sizeof (HARDWARE_STATUS))
#define IOW_MUTE_ON_EXTERNAL_SPEAKER    _IO(GPIO_DRIVER_MAGIC, 1)
#define IOW_MUTE_OFF_EXTERNAL_SPEAKER   _IO(GPIO_DRIVER_MAGIC, 2)
#define IOR_OUT_PIN_STATUS              _IOR(GPIO_DRIVER_MAGIC, 3, \
                                            sizeof (UINT32))
#define IOW_OUT_PIN_STATUS              _IOW(GPIO_DRIVER_MAGIC, 4, \
                                            sizeof (UINT32))
#define IOW_USB_OFF                     _IO(GPIO_DRIVER_MAGIC, 5)
#define IOW_USB_ON                      _IO(GPIO_DRIVER_MAGIC, 6)

/* battery charge status */

#define CHARGE_STATE_ACPWR_OFF      0 /* AC power is off */
#define CHARGE_STATE_DISABLED       1 /* charger disabled */
#define CHARGE_STATE_FAULT          2 /* charge fault */
#define CHARGE_STATE_COMPLETE       3 /* charging complete */
#define CHARGE_STATE_FAST_CHARGING  4 /* Fast charging */
#define CHARGE_STATE_TOPPINGUP      5 /* topping up */

#define AC_POWER                    (1 << 7)
#define CHARGE_FAULT                (1 << 3)
#define BATTERY_CHARGING            (1 << 3)

#define DISABLE_EXTERNAL_SPEAKER    (1 << 19)
#define ENABLE_EXTERNAL_SPEAKER     (~DISABLE_EXTERNAL_SPEAKER)

#define SET_RF_POWER_ON             (1 << 20)
#define SET_EXT_SPEAKER             (1 << 19)
#define SET_CHARGER_SHUTDOWN        (1 << 12)
#define SET_INT_SPEAKER             (1 << 6)
#define SET_CHARGE_OUT              (1 << 2)

#define CHARGE_OUT_PULLUP_DISABLE   (1 << 2)

#define USB_INT_LEVEL_CLEAR			~(7 << 4)
#define USB_INT_LEVEL_BOTH_EDGE		(6 << 4)
#define CONFIG_USB_INPUT			~(3 << 2)
#define CONFIG_USB_INT				(2 << 2)
#define USB_INPUT_LEVEL				(1 << 1)
#define EXT_POWER_PIN				(1 << 7)


#define RF_POWER_ON_MASK			0x00000001 /* Mask for refernce voltage */
#define MUTE_EXT_SPEAKER_MASK       0x00000002 /* Mask for External speaker */
#define CHARGER_SHUTDOWN_MASK       0x00000004 /* Mask for chager shutdown */
#define MUTE_INT_SPEAKER_MASK       0x00000008 /* Mask for internal speaker */
#define CHARGE_OUT_MASK             0x00000010 /* mask for charge out pin */

#define CHARGE_OUT_CON_MASK         (3 << 4)/* Mask for charge out pin */
#define CHARGE_OUT_CON_OUTPUT       (1 << 4)/* configure charge out as output */

/* Bit mask for HARDWARE_STATUS->u8InputStatus */

#define ONOFF_MASK                  0x01 /* Mask for On-off bu*/
#define USB_DETECT_MASK             0x02 /* Mask for USB power detect */
#define IGNITION_MASK               0x04 /* Mask for Igntion detection */
#define EXT_POWER_MASK              0x08 /* Mask for external power*/
#define DOCK_SENSE0_MASK            0x10 /* Mask for Docking sense0 */
#define DOCK_SENSE1_MASK            0x20 /* Mask for Docking sense1 */

#define IRQ_ONOFF                   IRQ_EINT0
#define IRQ_USB_DETECT              IRQ_EINT1
#define IRQ_IGNITION                IRQ_EINT7
#define IRQ_DOCK_SENSE0             IRQ_EINT13
#define IRQ_DOCK_SENSE1             IRQ_EINT14
#define IRQ_EXT_POWER               IRQ_EINT15

#define POWER_OFF               0x00 /* device is powered off */
#define POWER_ON                0x01 /* device iis powered on */

#define ONOFF_BUTTON		0x1
#define ONOFF_INTERRUPT     2
#define ONOFF_INPUT	        3

#define ACPWR_INTERRUPT     (2 << 14)
#define ACPWR_INPUT         (3 << 14)

/* typedefs */

typedef struct hardwareStatus /* Hardware status structure */
{

/*UINT8 u8DockStatus;  */ /*Bit[0] - DOCKSNS0, Bit[1] - DOCKSNS0*/

UINT8 u8ChargeStatus; /* charge state */

/* 
* Bit[0] - on/off, BIT[1] - USB detect BIT[2] - status of ignition pin, 
* BIT[3] - Ext. power detect Bit[4] - DOCKSNS0, Bit[5] - DOCKSNS0
*/

UINT8 u8InputStatus;
}HARDWARE_STATUS;

#ifdef __cplusplus
} 
#endif /* __cplusplus */ 
 
#endif /* __INCBarc_Gpioh */
