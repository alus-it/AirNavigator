//============================================================================
// Name        : TsScreen.c
// Since       : 3/11/2013
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 3/11/2013
// Description : Touch screen manager
//============================================================================

#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <barcelona/Barc_ts.h>
#include <barcelona/Barc_gps.h>
#include <barcelona/Barc_Battery.h>
#include "TsScreen.h"
#include "AirNavigator.h"

static int tsfd=-1;
static fd_set fdset;
static int maxfd;
static int gps;

void TsScreen_Init() {
	tsfd=open("/dev/ts",O_RDWR|O_NOCTTY|O_NONBLOCK);
	if(tsfd<0) tsfd=open("/dev/ts",O_RDONLY|O_NOCTTY|O_NONBLOCK);
	if(tsfd<0) {
		logText("tomtom_touchscreen: Kon niet geopend worden '/dev/ts'.\n");
		tsfd=-1;
	}
	ioctl(tsfd,TS_SET_RAW_OFF,NULL);
	maxfd=tsfd+1;
	FD_ZERO(&fdset);
	FD_SET(tsfd, &fdset);
}

void TsScreen_Exit() {
	if(tsfd>=0) close(tsfd);
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
	static TS_EVENT event;
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
