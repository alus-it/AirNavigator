//============================================================================
// Name        : BlackBox.h
// Since       : 14/9/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 23/11/2013
// Description : Produces tracelogFiles as XML GPX files
//============================================================================

#ifndef BLACKBOX_H_
#define BLACKBOX_H_

#define MIN_DIST 0.00000109872 // 7 m in Rad

enum blackBoxStatus {
	BBS_NOT_SET,
	BBS_WAIT_FIX,
	BBS_WAIT_POS,
	BBS_WAIT_OPT,
	BBS_PAUSED
};

void BlackBoxStart(void);
short BlackBoxRecordPos(double lat, double lon, float timestamp, int hour, int min, float sec, short dateChanged);
short BlackBoxRecordAlt(double altMt);
short BlackBoxRecordSpeed(double speedMTSec);
short BlackBoxRecordCourse(double course);
short BlackBoxCommit(void);
void BlackBoxPause(void);
void BlackBoxResume(void);
void BlackBoxClose(void);

#endif /* BLACKBOX_H_ */
