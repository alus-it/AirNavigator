//============================================================================
// Name        : Ephemerides.h
// Since       : 2/8/2014
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : https://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2020 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/alus-it/AirNavigator.git
// Last change : 2/8/2013
// Description : Functions to manage sunrise and sunset times
//============================================================================

#ifndef EPHEMERIDES_H_
#define EPHEMERIDES_H_

#include "Common.h"


enum sunriseSunsetCases { //Sunrise/sunset algorithm return values
	SUN_RISES_AND_SETS,
	SUN_NEVER_RISES,    //The sun never rises on this location (on the specified date)
	SUN_NEVER_SETS      //The sun never sets on this location (on the specified date)
};

struct EphemeridesData {
	enum sunriseSunsetCases situation;
	int sunriseHour;
	int sunsetHour;
	int sunriseMin;
	int sunsetMin;
	float sunriseSec;
	float sunsetSec;
};

struct EphemeridesStruct {
	struct EphemeridesData deparure;
	struct EphemeridesData currentPosition;
	struct EphemeridesData destination;
};

struct EphemeridesStruct Ephemerides;

int calcSunriseSunset(double lat, double lon, int day, int month, int year, double zenith, int localOffset, double *riseTime, double *setTime);
int calcSunriseSunsetWithInternalClockTime(double lat, double lon, double *riseTime, double *setTime);
void calcFlightPlanEphemerides(double lat, double lon, bool isDeparture);

#endif /* EPHEMERIDES_H_ */
