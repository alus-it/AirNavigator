//============================================================================
// Name        : BlackBox.h
// Since       : 14/9/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 03/12/2011
// Description : Produces tracelogFiles as XML GPX files
//============================================================================

#ifndef BLACKBOX_H_
#define BLACKBOX_H_

#define BBS_NOT_SET   0
#define BBS_WAIT_FIX  1
#define BBS_WAIT_POS  2
#define BBS_WAIT_OPT  3
#define BBS_PAUSED    4

#define MIN_DIST 0.00000109872 // 7 m in Rad

void BlackBoxStart();
short BlackBoxRecordPos(double lat, double lon, float timestamp, int hour, int min, float sec);
short BlackBoxRecordAlt(double altMt);
short BlackBoxRecordSpeed(double speedMTSec);
short BlackBoxRecordCourse(double course);
short BlackBoxCommit();
void BlackBoxPause();
void BlackBoxResume();
void BlackBoxClose();

#endif /* BLACKBOX_H_ */
