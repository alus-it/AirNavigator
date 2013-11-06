//============================================================================
// Name        : TsScreen.c
// Since       : 3/11/2013
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 5/11/2013
// Description : Touch screen manager
//============================================================================

#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <barcelona/Barc_ts.h>
#include <barcelona/Barc_gps.h>
#include <barcelona/Barc_Battery.h>
#include "TsScreen.h"
#include "AirNavigator.h"

volatile short readingTS=-1; //-1 means still not initialized
static int tsfd=-1;
pthread_t threadTsScreen;
condVar_t condVar;
TS_EVENT lastTouch;
static int gps;


void initializeTsScreen() {
	condVar=(condVar_t)malloc(sizeof(struct condVarStruct));
	pthread_mutex_init(&condVar->lastTouchMutex,NULL);
	pthread_cond_init(&condVar->lastTouchSignal,NULL);

	tsfd=open("/dev/ts",O_RDONLY|O_NOCTTY|O_NONBLOCK);
	if(tsfd<0) {
		logText("TsScreen: ERROR can't open the device: /dev/ts\n");
		tsfd=-1;
		readingTS=-1;
	}
	ioctl(tsfd,TS_SET_RAW_OFF,NULL);
	readingTS=0;
}

void* runTsScreen(void *ptr) {
	if(tsfd<0) {
		readingTS=0;
		pthread_exit(NULL);
		return NULL;
	}
	fd_set fdset;
	int maxfd=tsfd+1;
	int read_len;
	while(readingTS) { // loop while waiting for input
		FD_ZERO(&fdset);
		FD_SET(tsfd, &fdset);
		select(maxfd,&fdset,NULL,NULL,NULL);
		TS_EVENT event;
		read_len=read(tsfd,&event,sizeof(TS_EVENT));
		if(read_len==sizeof(TS_EVENT)) {
			if(event.pressure==0) {
				pthread_mutex_lock(&condVar->lastTouchMutex);
				lastTouch=event;
				pthread_cond_signal(&condVar->lastTouchSignal);
				pthread_mutex_unlock(&condVar->lastTouchMutex);
			}
		}
	}
	pthread_exit(NULL);
	return NULL;
}

short TsScreenStart() {
	if(readingTS==-1) initializeTsScreen();
	if(readingTS==0) {
		readingTS=1;
		if(pthread_create(&threadTsScreen,NULL,runTsScreen,(void*)NULL)) {
			readingTS=0;
			logText("TsScreen: ERROR unable to create the reading thread.\n");
		}
	}
	return readingTS;
}

void TsScreenClose() {
	if(readingTS==1) readingTS=0;
	pthread_join(threadTsScreen,NULL); //wait for thread death
	pthread_mutex_destroy(&condVar->lastTouchMutex);
	pthread_cond_destroy(&condVar->lastTouchSignal);
	free(condVar);
	if(tsfd>=0) close(tsfd);
	readingTS=-1;
}

condVar_t TsScreenGetCondVar() {
	return condVar;
}

TS_EVENT TsScreenGetLastTouch() {
	return lastTouch;
}

int TsScreen_pen(int *x, int *y, int *pen) {
	if(tsfd<0) return 0;
	int read_len;
	TS_EVENT new_event;
	static int have_previous=0;
	static TS_EVENT prev_event;
	if(!have_previous) {
		read_len=read(tsfd,&prev_event,sizeof(TS_EVENT));
		if(read_len==sizeof(TS_EVENT)) have_previous=1;
	}
	if(!have_previous) return 0; // if we still don't have an event, there are no events pending, and we can just return
	memcpy(&new_event,&prev_event,sizeof(TS_EVENT)); // We have an event
	have_previous=0;
	read_len=read(tsfd,&prev_event,sizeof(TS_EVENT));
	if(read_len==sizeof(TS_EVENT)) have_previous=1;
	while(have_previous&&(prev_event.pressure!=0)==(new_event.pressure!=0)) {
		memcpy(&new_event,&prev_event,sizeof(TS_EVENT));
		have_previous=0;
		read_len=read(tsfd,&prev_event,sizeof(TS_EVENT));
		if(read_len==sizeof(TS_EVENT)) have_previous=1;
	}
	*x=new_event.x;
	*y=new_event.y;
	*pen=new_event.pressure;
	return 1;
}

int TsScreen_touch(int *x, int *y) {
	if(tsfd<0) return 0;
	int read_len;
	TS_EVENT event;
	read_len=read(tsfd,&event,sizeof(TS_EVENT));
	if(read_len==sizeof(TS_EVENT)) {
		if(event.pressure==0) {
			*x=event.x;
			*y=event.y;
			return 1;
		}
	}
	return 0;
}


short checkBattery(short *batVolt, short *refVolt, short *chargeCurr) {
	int battery=open("/dev/battery",O_RDONLY|O_NOCTTY);
	if(battery<0) return 0;
	else {
		BATTERY_STATUS bs;
		if(ioctl(battery,IOR_BATTERY_STATUS,&bs)==-1) { //if the return values is -1 it means fault
			close(battery);
			return -1;
		}
		*batVolt=(short)bs.u16BatteryVoltage;   // battery voltage
		*refVolt=(short)bs.u16ReferenceVoltage; // reference voltage
		*chargeCurr=(short)bs.u16ChargeCurrent; // Charge current
		//What kind of units do we have here?
		close(battery);
	}
	return 1;
}

short enableGPS() {
	gps=open("/dev/gps",O_RDWR);
	if(gps<0) return 0;
	else {
		if(ioctl(gps,IOW_GPS_ON,NULL)==-1) { //if the return values is -1 it means fault
			close(gps);
			gps=-1;
			return -1;
		}
	}
	return 1;
}

void disableGPS() {
	if(gps>=0) close(gps);
}
