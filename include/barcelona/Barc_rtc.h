/* include/barcelona/Barc_rtc.h
 *
 * Real-time clock device driver for Barcelona.
 * Copyright (C) 2003,2004 TomTom BV.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * Major number is now 242 which *seems* to be unused by Barcelona - someone want to devfs'ize it?
 *
 * Here's how it might be used
 * mount -o rw,remount /dev/sdcard
 * insmod rtc.o
 * mknod /dev/rtc c 242 100
 * chmod a+rw /dev/rtc
 * cat /dev/rtc <--- returns sequence of 6 (plain binary) bytes: (YY, MM, DD, HH, MM, SS)
 * echo <a_binary_string_of_6_bytes> > /dev/rtc
 * rmmod rtc
 */

#ifndef __BARC_RTC_H__
#define __BARC_RTC_H__

#define     RTC_DEVNAME         "rtc"
#define     RTC_MAJOR			242 //MISC_MAJOR

extern unsigned long s3c2410_get_rtc_time (void);
extern void s3c2410_set_rtc_time (unsigned long totalSeconds);

#endif
