//============================================================================
// Name        : TSreader.c
// Since       : 3/11/2013
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 28/11/2013
// Description : Touch screen reader
//============================================================================

#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <barcelona/Barc_ts.h>
//#include <barcelona/Barc_Battery.h>
#include "TSreader.h"
#include "Common.h"

struct TSreaderStruct {
	volatile short reading;
	int tsfd;
	pthread_t thread;
	condVar_t condVar;
	TS_EVENT lastTouch;
};

void TSreaderRelease(void);
void initializeTSreader(void);
void* runTSreader(void *arg);

static struct TSreaderStruct TSreader = {
	.reading=-1, //-1 means still not initialized
	.tsfd=-1,
	.condVar=NULL
};

void TSreaderRelease(void) {
	pthread_mutex_destroy(&TSreader.condVar->lastTouchMutex);
	pthread_cond_destroy(&TSreader.condVar->lastTouchSignal);
	free(TSreader.condVar);
	if(TSreader.tsfd>=0) close(TSreader.tsfd);
	TSreader.tsfd=-1;
	TSreader.reading=-1;
}

void initializeTSreader(void) {
	TSreader.condVar=(condVar_t)malloc(sizeof(struct condVarStruct));
	if(pthread_mutex_init(&TSreader.condVar->lastTouchMutex,NULL) || pthread_cond_init(&TSreader.condVar->lastTouchSignal,NULL)) {
		printLog("TSreader: ERROR while initializing mutex and condition variable\n");
		TSreaderRelease();
	}
	TSreader.tsfd=open("/dev/ts",O_RDONLY|O_NOCTTY|O_NONBLOCK);
	if(TSreader.tsfd<0) {
		printLog("TSreader: ERROR can't open the device: /dev/ts\n");
		TSreaderRelease();
	}
	ioctl(TSreader.tsfd,TS_SET_RAW_OFF,NULL);
	TSreader.reading=0; //in this case we can start the thread
}

void* runTSreader(void *arg) { //This is the thread listening for input on the touch screen
	struct sigaction sa;
	sa.sa_handler = NULL;
	sa.sa_sigaction = NULL/*gotsig*/;
	sa.sa_flags = SA_SIGINFO;
	sigemptyset(&sa.sa_mask);
	fd_set fdset;
	int maxfd=TSreader.tsfd+1;
	int read_len;
	while(TSreader.reading) { //loop while waiting for input
		FD_ZERO(&fdset);
		FD_SET(TSreader.tsfd, &fdset);
		if(sigaction((int)arg, &sa, NULL)<0) { //register signal to kill the thread otherwise it will wait until next input
			printLog("TSreader: ERROR while registering signal to stop listen thread.");
			TSreader.reading=0;
			pthread_exit(NULL);
			return NULL;
		}
		select(maxfd,&fdset,NULL,NULL,NULL); //wait until we have something new
		TS_EVENT event;
		read_len=read(TSreader.tsfd,&event,sizeof(TS_EVENT));
		if(read_len==sizeof(TS_EVENT)) {
			if(event.pressure==0) { //to detect when the finger is going away from the screen so the touch is completed
				pthread_mutex_lock(&TSreader.condVar->lastTouchMutex);
				TSreader.lastTouch=event; //it's a simple struct no need to use memcpy
				pthread_cond_signal(&TSreader.condVar->lastTouchSignal); //signal to the main that there is a new touch
				pthread_mutex_unlock(&TSreader.condVar->lastTouchMutex);
			}
		}
	}
	pthread_exit(NULL);
	return NULL;
}

short TSreaderStart() {
	if(TSreader.reading==-1) initializeTSreader();
	if(TSreader.reading==0) {
		TSreader.reading=1;
		if(pthread_create(&TSreader.thread,NULL,runTSreader,(void*)SIGUSR1)) {
			TSreader.reading=0;
			printLog("TSreader: ERROR unable to create the reading thread.\n");
			TSreaderRelease();
		}
	}
	return TSreader.reading;
}

void TSreaderClose() {
	if(TSreader.reading==1) {
		TSreader.reading=0;
		pthread_kill(TSreader.thread,SIGUSR1); //send the signa to kill the thread
		pthread_join(TSreader.thread,NULL); //wait for thread death
	}
	TSreaderRelease();
}

condVar_t TSreaderGetCondVar() {
	return TSreader.condVar;
}

TS_EVENT TSreaderGetLastTouch() {
	return TSreader.lastTouch;
}

/*
//Other useful function for tomtom devices but not working
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

*/
