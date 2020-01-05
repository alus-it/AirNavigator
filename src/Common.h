//============================================================================
// Name        : Common.h
// Since       : 3/12/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : https://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2016 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 8/12/2013
// Description : Common definitions of AirNavigator
//============================================================================

#ifndef COMMON_H_
#define COMMON_H_

#define BASE_PATH "/mnt/sdcard/AirNavigator/"

typedef char bool;

enum boolean { false, true };

enum mainStatus {
	MAIN_NOT_INIT,
	MAIN_DISPLAY_MENU,
	MAIN_DISPLAY_SELECT_ROUTE,
	MAIN_DISPLAY_HSI,
	MAIN_DISPLAY_SUNRISE_SUNSET
};

bool openLog(void);
int printLog(const char *texts, ...);
void closeLog(void);
float getCurrentTime(void);
enum mainStatus getMainStatus(void);

#endif
