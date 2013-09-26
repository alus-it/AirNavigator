/* include/barcelona/Bard_wd.h
 *
 * Watchdog timer device driver for Barcelona.
 * Copyright (C) 2004 TomTom BV.
 * Author: Koen Martens (kmartens@sonologic.nl)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * Description:
 * Reimplementation of the watchdog timer using the driver skeleton
 * from the rtc driver.
 *
 * Usage:
 * - Create the proper device 'mknod /dev/watchdog c 246 0'
 * - Set watchdog timeout with ioctl on /dev/watchdog (IOW_WD_SET_TIME, in seconds)
 * - Kick the watchdog within the set time using IOW_WD_KICK (NULL argument)
 */

#ifndef __BARC_WD_H__
#define __BARC_WD_H__

#include <linux/ioctl.h>

#define     WD_DEVNAME              "watchdog"
#define     WD_MAJOR                246

/* probably not very magic */
#define     ACC_DRIVER_MAGIC        'A'

#define     IOW_WD_SET_TIME         _IO(ACC_DRIVER_MAGIC, 2)
#define     IOW_WD_KICK             _IO(ACC_DRIVER_MAGIC, 3)

#endif /* __BARC_WD_H__ */
