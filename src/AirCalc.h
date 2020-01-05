//============================================================================
// Name        : AirCalc.h
// Since       : 19/2/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : https://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2016 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 28/2/2016
// Description : Header of AirCalc.c
//============================================================================

#ifndef AIRCALC_H_
#define AIRCALC_H_

#include <math.h>
#include "Common.h"

//Common calculation defines
#define TWO_PI  (2*M_PI)
#define SIXTYTH 0.01666666666666666667 // 1/60
#define MILE_FT 5280 //1 Mile = 5280 Ft (1760 Yards)


double Km2Nm(const double valueKm);
double Nm2Km(const double valueNm);
double Km2Miles(const double valueKm);
double Miles2Km(const double valueMiles);
double m2Ft(const double valueMt);
double m2Nm(const double valueMt);
double Ft2m(const double valueFt);
double Kmh2ms(const double valueKmh);
double ms2Kmh(const double valueMs);
double FtMin2ms(const double valueFtMin);
double ms2FtMin(const double valueMs);
double calcTotalSpeed(const double hSpeed, const double vSpeed);
double latDegMin2rad(const int degrees, const float minutes, const bool N);
double lonDegMin2rad(const int degrees, const float minutes, const bool E);
double latDegMinSec2rad(const int deg, const int min, const float sec, const bool N);
double lonDegMinSec2rad(const int deg, const int min, const float sec, const bool E);
double calcAngularDist(const double lat1, const double lon1, const double lat2, const double lon2);
double calcSmallAngularDist(const double lat1, const double lon1, const double lat2, const double lon2);
double Rad2Km(const double rad);
double Km2Rad(const double km);
double Rad2m(const double rad);
double m2Rad(const double mt);
double Rad2Nm(const double rad);
double Rad2Mi(const double rad);
double Rad2Deg(const double rad);
double Deg2Rad(const double deg);
double absAngle(const double angle);
//double calcRhumbLineRoute(const double lat1, const double lon1, const double lat2, const double lon2, double *d);
double calcGreatCircleRoute(const double lat1, const double lon1, const double lat2, const double lon2, double *d);
double calcGreatCircleCourse(const double lat1, const double lon1, const double lat2, const double lon2);
double calcGreatCircleFinalCourse(const double lat1, const double lon1, const double lat2, const double lon2);
bool calcIntermediatePoint(const double lat1, const double lon1, const double lat2, const double lon2, const double atd, const double d, double *latI, double *lonI);
bool isAngleBetween(double low, double angle, double hi);
double calcGCCrossTrackError(const double lat1, const double lon1, const double lon2, const double latX, const double lonX, const double course12, double *atd);
void convertDecimal2DegMin(const double dec, int *deg, double *min);
void convertDecimal2DegMinSec(const double dec, int *deg, int *min, float *sec);
void convertRad2DegMinSec(const double rad, int *deg, int *min, float *sec);
void convertTimestamp2HourMinSec(const float timestamp, int *hour, int *min, float *sec);
double calcWindDirSpeed(const double crs, const double hd, const double tas, const double gs, double *ws);
int calcHeadingGroundSpeed(const double crs, const double wd, const double tas, const double ws, double *hd, double *gs);
double calcCourseGroundSpeed(const double hd, const double wd, const double tas, const double ws, double *gs);
void calcHeadCrossWindComp(const double ws, const double wd, const double rd, double *hw, double *xw);
void calcBisector(double currCourse, const double nextCourse, double *bisector, double *bisectorOpposite);
bool bisectorOverpassed(const double currCourse, const double actualCurrCourse, const double bisector1, const double bisector2);

#endif
