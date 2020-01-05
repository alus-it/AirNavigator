//============================================================================
// Name        : Common.c
// Since       : 3/12/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : https://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2020 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/alus-it/AirNavigator.git
// Last change : 23/11/2013
// Description : Common functions of AirNavigator
//============================================================================

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <pthread.h>
#include "Common.h"


static pthread_mutex_t logMutex=PTHREAD_MUTEX_INITIALIZER;
static FILE *logFile=NULL;

bool openLog(void) {
	char *logPath;
	asprintf(&logPath,"%slog.txt",BASE_PATH);
	logFile=fopen(logPath,"w"); //create log file
	free(logPath);
	if(logFile!=NULL) return true;
	else return false;
}

int printLog(const char *texts, ...) {
	int done=-1;
	if(logFile!=NULL) {
		va_list arg;
		va_start(arg,texts);
		pthread_mutex_lock(&logMutex);
		done=vfprintf(logFile,texts,arg);
		pthread_mutex_unlock(&logMutex);
		va_end(arg);
		return done;
	}
	return done;
}

void closeLog(void) {
	if(logFile!=NULL) fclose(logFile);
}

float getCurrentTime(void) {
	struct tm time_str;
	long currTime=time(NULL); //get current time
	time_str=*localtime(&currTime);
	int n=time_str.tm_hour; //hour
	float timeVal=n*3600;
	n=time_str.tm_min; //minute
	timeVal+=n*60;
	n=time_str.tm_sec;
	timeVal+=n;
	return timeVal;
}
