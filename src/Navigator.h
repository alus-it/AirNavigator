//============================================================================
// Name        : Navigator.h
// Since       : 19/2/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 25/12/2012
// Description : Navigation manager
//============================================================================

#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#define STATUS_NOT_INIT			0
#define STATUS_NO_ROUTE_SET	1
#define STATUS_TO_START_NAV	2
#define STATUS_WAIT_FIX			3
#define STATUS_NAV_TO_DPT		4
#define STATUS_NAV_TO_WPT		5
#define STATUS_END_NAV			6
#define STATUS_NAV_BUSY			7

typedef struct wp {
	int seqNo;						//sequence number
	char *name;						//name of the WP
	double latitude;			//rad
	double longitude;			//rad
	double altitude;			//meter
	double dist;					//distance to to this WP in rad
	double trueCourse;		//initial true course to to this WP in rad
	double arrTimestamp;	//arrival to this WP timestamp in seconds (from h 0:00)
	struct wp *prev;			//Pointer to the previous waypoint in the list
	struct wp *next;			//Pointer to the next waypoint in the list
} *wayPoint;

int NavLoadFlightPlan(char* GPXfile);
void NavAddWayPoint(double latWP, double lonWP, double altWP, char *WPname);
void NavClearRoute();
void NavUpdatePosition(double lat, double lon, double alt, double speed, double dir, float timestamp);
short checkDaytime();
void NavStartNavigation(float timestamp);
int NavReverseRoute();
void NavSkipCurrentWayPoint();
float getCurrentTime();
void NavClose();

#endif /*NAVIGATOR_H_*/
