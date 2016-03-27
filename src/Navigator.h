//============================================================================
// Name        : Navigator.h
// Since       : 19/2/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2016 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 28/2/2016
// Description : Header of the navigation manager: Navigator.c
//============================================================================

#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include "Common.h"

enum navigatorStatus {
	NAV_STATUS_NOT_INIT,
	NAV_STATUS_NO_ROUTE_SET,
	NAV_STATUS_NAV_BUSY,
	NAV_STATUS_TO_START_NAV,
	NAV_STATUS_WAIT_FIX,
	NAV_STATUS_NAV_TO_DPT,
	NAV_STATUS_NAV_TO_WPT,
	NAV_STATUS_NAV_TO_DST,
	NAV_STATUS_NAV_TO_SINGLE_WP,
	NAV_STATUS_END_NAV
};

int NavLoadFlightPlan(char* GPXfile);
void NavAddWayPoint(double latWP, double lonWP, double altWP, char *WPname);
void NavRedrawNavInfo(void);
void NavRedrawEphemeridalInfo(void);
void NavClearRoute(void);
void NavUpdatePosition(double lat, double lon, double alt, double speed, float timestamp);
short checkDaytime(bool calcOnlyDest);
void NavStartNavigation(void);
int NavReverseRoute(void);
void NavSkipCurrentWayPoint(void);
enum navigatorStatus NavGetStatus(void);
void NavClose(void);

#endif /*NAVIGATOR_H_*/
