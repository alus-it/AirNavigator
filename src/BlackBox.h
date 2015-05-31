//============================================================================
// Name        : BlackBox.h
// Since       : 14/9/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2015 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 8/12/2013
// Description : Produces tracelogFiles as XML GPX files
//============================================================================

#ifndef BLACKBOX_H_
#define BLACKBOX_H_

#include "Common.h"

void BlackBoxStart(void);
bool BlackBoxIsStarted(void);
bool BlackBoxRecordPos(double lat, double lon, float timestamp, int hour, int min, float sec, int day, int month, int year, bool dateChanged);
bool BlackBoxRecordAlt(double altMt);
bool BlackBoxRecordSpeed(double speedMTSec);
bool BlackBoxRecordCourse(double course);
bool BlackBoxCommit(void);
void BlackBoxPause(void);
bool BlackBoxIsPaused(void);
void BlackBoxResume(void);
void BlackBoxClose(void);

#endif /* BLACKBOX_H_ */
