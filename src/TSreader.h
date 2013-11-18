//============================================================================
// Name        : TSreader.h
// Since       : 3/11/2013
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 12/11/2013
// Description : Header of TSreader.c the touch screen reader
//============================================================================

#ifndef TSREADER_H_
#define TSREADER_H_

#include <pthread.h>
#include <barcelona/Barc_ts.h>

typedef struct condVarStruct {
	pthread_mutex_t lastTouchMutex;
	pthread_cond_t lastTouchSignal;
} *condVar_t;

short TSreaderStart(void);
void TSreaderClose(void);
condVar_t TSreaderGetCondVar(void);
TS_EVENT TSreaderGetLastTouch(void);

/*
short checkBattery(short *batVolt, short *refVolt, short *chargeCurr);
short enableGPS(void);
void disableGPS(void);
*/

#endif /* TSREADER_H_ */
