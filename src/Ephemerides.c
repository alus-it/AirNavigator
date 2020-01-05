//============================================================================
// Name        : Ephemerides.c
// Since       : 2/8/2014
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : https://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2020 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/alus-it/AirNavigator.git
// Last change : 2/8/2013
// Description : Functions to manage sunrise and sunset times
//============================================================================

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include "Ephemerides.h"
#include "AirCalc.h"
#include "Configuration.h"
#include "GPSreceiver.h"


//struct EphemeridesStruct Ephemerides = {
	//.departurePresent = false
//};

int calcSunriseSunset(double lat, double lon, int day, int month, int year, double zenith, int localOffset, double *riseTime, double *setTime) {
	int retVal=SUN_RISES_AND_SETS;
	lon=-lon; //Beacuse here longitudes to E are to be considered as positive
	double N=floor(275*month/9)-(floor((month+9)/12)*(1+floor((year-4*floor(year/4)+2)/3)))+day-30;
	double lngHour=Rad2Deg(lon)/15;
	double localT=-1;
	short i=0;
	for(i=0;i<2;i++) {
		double t;
		if(i==0) t=N+((6-lngHour)/24);  //Rising
		else t=N+((18-lngHour)/24); //Setting
		double M=0.9856*t-3.289;
		double L=M+(1.916*sin(Deg2Rad(M)))+(0.020*sin(Deg2Rad(2*M)))+282.634;
		if(L<0) L+=360;
		else if(L>360) L-=360;
		double RA=Rad2Deg(absAngle(atan(0.91764*tan(Deg2Rad(L)))));
		double Lquadrant=(floor(L/90))*90;
		double RAquadrant=(floor(RA/90))*90;
		RA+=Lquadrant-RAquadrant;
		RA=RA/15;
		double sinDec=0.39782*sin(Deg2Rad(L));
		double cosDec=cos(asin(sinDec));
		double cosH=(cos(zenith)-(sinDec*sin(lat)))/(cosDec*cos(lat));
		if(cosH>=-1 && cosH<=1) { //to avoid cases where the sun never rises or never sets
			double H;
			if(i==0) H=360-Rad2Deg(acos(cosH)); //rising
			else H=Rad2Deg(acos(cosH)); //setting
			H=H/15;
			double T=H+RA-(0.06571*t)-6.622;
			double UT=T-lngHour;
			localT=UT+localOffset;
			if(localT<0) localT+=24;
			else if(localT>24) localT-=24;
		} else {
			if(i==0) retVal=SUN_NEVER_RISES;
			else retVal=SUN_NEVER_SETS;
		}
		if(i==0) *riseTime=localT;
		else *setTime=localT;
	}
	return retVal;
}

int calcSunriseSunsetWithInternalClockTime(double lat, double lon, double *riseTime, double *setTime) {
	struct tm time_str;
	long timestamp=time(NULL); //get current time
	time_str=*localtime(&timestamp); //to obtain day, month, year, hour and minute
	int year=time_str.tm_year+1900; //year
	int month=time_str.tm_mon+1; //month
	int day=time_str.tm_mday; //day
	return calcSunriseSunset(lat,lon,day,month,year,config.sunZenith,config.timeZone,riseTime,setTime);
}

void calcFlightPlanEphemerides(double lat, double lon, bool isDeparture) {
	double riseTime, setTime;
	if(gps.fixMode>MODE_NO_FIX) {
		//TODO: can we avoid to lock the mutex here?
		pthread_mutex_lock(&gps.mutex);
		calcSunriseSunset(lat,lon,gps.day,gps.month,gps.year,config.sunZenith,0,&riseTime,&setTime);
		pthread_mutex_unlock(&gps.mutex);
	}
	else calcSunriseSunsetWithInternalClockTime(lat,lon,&riseTime,&setTime);
	if(isDeparture) {
		//Ephemerides.departurePresent=true;
		convertDecimal2DegMinSec(riseTime,&Ephemerides.deparure.sunriseHour,&Ephemerides.deparure.sunriseMin,&Ephemerides.deparure.sunriseSec);
		convertDecimal2DegMinSec(setTime,&Ephemerides.deparure.sunsetHour,&Ephemerides.deparure.sunsetMin,&Ephemerides.deparure.sunsetSec);
	} else {
		convertDecimal2DegMinSec(riseTime,&Ephemerides.destination.sunriseHour,&Ephemerides.destination.sunriseMin,&Ephemerides.destination.sunriseSec);
		convertDecimal2DegMinSec(setTime,&Ephemerides.destination.sunsetHour,&Ephemerides.destination.sunsetMin,&Ephemerides.destination.sunsetSec);
	}
}

