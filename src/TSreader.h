//============================================================================
// Name        : TSreader.h
// Since       : 3/11/2013
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2014 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 29/11/2013
// Description : Header of TSreader.c the touch screen reader
//============================================================================

#ifndef TSREADER_H_
#define TSREADER_H_

#include <pthread.h>
#include <barcelona/Barc_ts.h>


short TSreaderStart(void);
void TSreaderClose(void);
short TSreaderGetTouch(TS_EVENT *lastTouch);

//short checkBattery(short *batVolt, short *refVolt, short *chargeCurr);

#endif /* TSREADER_H_ */
