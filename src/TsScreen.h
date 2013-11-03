//============================================================================
// Name        : TsScreen.h
// Since       : 3/11/2013
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 3/11/2013
// Description : Header of TsScreen.c the touch screen manager
//============================================================================

#ifndef TSSCREEN_H_
#define TSSCREEN_H_

void TsScreen_Init();
void TsScreen_Exit();
int TsScreen_pen(int *x, int *y, int *pen);
int TsScreen_touch(int *x, int *y);

short checkBattery(short *batVolt, short *refVolt, short *chargeCurr);
short enableGPS();
void disableGPS();

#endif /* TSSCREEN_H_ */
