//============================================================================
// Name        : AirCalc.h
// Since       : 19/2/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 9/11/2013
// Description : Header of AirCalc.c
//============================================================================

#ifndef AIRCALC_H_
#define AIRCALC_H_

#include <math.h>

//Common calculation defines
#define TWO_PI  (2*M_PI)
#define SIXTYTH 0.01666666666666666667 // 1/60
#define MILE_FT 5280 //1 Mile = 5280 Ft (1760 Yards)

//Sunrise/sunset algorithm return values
#define SUN_RISES_AND_SETS 1
#define SUN_NEVER_RISES    2 //The sun never rises on this location (on the specified date)
#define SUN_NEVER_SETS     3 //The sun never sets on this location (on the specified date)

double Km2Nm(double valueKm);
double Nm2Km(double valueNm);
double Km2Miles(double valueKm);
double Miles2Km(double valueMiles);
double m2Ft(double valueMt);
double m2Nm(double valueMt);
double Ft2m(double valueFt);
double Kmh2ms(double valueKmh);
double ms2Kmh(double valueMs);
double FtMin2ms(double valueFtMin);
double ms2FtMin(double valueMs);
double calcTotalSpeed(double hSpeed,double vSpeed);
double latDegMin2rad(int degrees,float minutes,short N);
double lonDegMin2rad(int degrees,float minutes,short E);
double latDegMinSec2rad(int deg, int min, float sec, short N);
double lonDegMinSec2rad(int deg, int min, float sec, short E);
double calcAngularDist(double lat1, double lon1, double lat2, double lon2);
double calcSmallAngularDist(double lat1, double lon1, double lat2, double lon2);
double Rad2Km(double rad);
double Km2Rad(double km);
double Rad2m(double rad);
double m2Rad(double mt);
double Rad2Nm(double rad);
double Rad2Mi(double rad);
double Rad2Deg(double rad);
double Deg2Rad(double deg);
double calcRhumbLineRoute(double lat1, double lon1, double lat2, double lon2, double *d);
double calcGreatCircleRoute(double lat1, double lon1, double lat2, double lon2, double *d);
double calcGreatCircleCourse(double lat1, double lon1, double lat2, double lon2);
double calcGreatCircleFinalCourse(double lat1, double lon1, double lat2, double lon2);
int calcIntermediatePoint(double lat1, double lon1, double lat2, double lon2, double atd, double d, double *latI, double *lonI);
short isAngleBetween(double low, double angle, double hi);
double calcGCCrossTrackError(double lat1, double lon1, double lon2, double latX, double lonX, double course12, double *atd);
int calcSunriseSunset(double lat, double lon, int day, int month, int year, double zenith, int localOffset, double *riseTime, double *setTime);
void convertDecimal2DegMin(double dec, int *deg, double *min);
void convertDecimal2DegMinSec(double dec, int *deg, int *min, float *sec);
void convertRad2DegMinSec(double rad, int *deg, int *min, float *sec);
void convertTimestamp2HourMinSec(float timestamp, int *hour, int *min, float *sec);
double calcWindDirSpeed(double crs, double hd, double tas, double gs, double *ws);
int calcHeadingGroundSpeed(double crs, double wd, double tas, double ws, double *hd, double *gs);
double calcCourseGroundSpeed(double hd, double wd, double tas, double ws, double *gs);
void calcHeadCrossWindComp(double ws, double wd, double rd, double *hw, double *xw);
void calcBisector(double currCourse, double nextCourse, double *bisector, double *bisectorOpposite);
short bisectorOverpassed(double currCourse, double actualCurrCourse, double bisector1, double bisector2);

#endif
