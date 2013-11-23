//============================================================================
// Name        : Common.h
// Since       : 3/12/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 23/11/2013
// Description : Common definitions of AirNavigator
//============================================================================

#ifndef COMMON_H_
#define COMMON_H_

#define BASE_PATH "/mnt/sdcard/AirNavigator/"

enum boolean { false, true };
typedef char bool;

bool openLog(void);
int printLog(const char *texts, ...);
void closeLog(void);
float getCurrentTime(void);

#endif
