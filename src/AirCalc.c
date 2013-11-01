//============================================================================
// Name        : AirCalc.c
// Since       : 19/2/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 31/12/2013
// Description : Collection of functions for air navigation calculation
//============================================================================

//TODO: Verify that we are not getting NaN using: if(isnan(number))
//TODO: Verify that we are not getting infinity using: if(isinf(number))

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "AirCalc.h"

//Internal AirCalc calculation defines
#define EARTH_RADIUS_KM 6371
#define EARTH_RADIUS_MI 3958.755865744
#define EARTH_RADIUS_M  6371000
#define NM_KM           1.852    //1 Knots = 1.852 Km/h
#define NM_M            1852     //1 NM = 1852 m
#define MILE_KM         1.609344 //1 Mile = 1.609344 Km
#define FT_M            0.3048   //1 Ft = 0.3048 m
#define FTMIN_MSEC      0.00508  //1 Ft/min = 0.00508 m/s
#define YARD_M          0.9144   //1 yard = 0.9144 m (3 Ft)
#define SEC_HOUR        0.0002777777777777777777777777778 //1/3600
#define HALF_PI         (PI/2)
#define QUARTER_PI      (PI/4)
#define DEG2RAD         (PI/180)
#define RAD2DEG         (180/PI)
#define RAD2NM          ((180*60)/PI)

//TOL is a small number of order machine precision- say 1e-15 (here 1e-16).
//this is used in calcRhumbLineRoute to avoid 0/0 indeterminacies on E-W courses.
#define SQRT_TOL 1e-8 //sqrt(TOL)=sqrt(1e-16)=1e-8 (6.4 cm)


double Km2Nm(double valueKmh) {
	return valueKmh/NM_KM;
}

double Nm2Km(double valueNm) {
	return valueNm*NM_KM;
}

double Km2Miles(double valueKm) {
	return valueKm/MILE_KM;
}

double Miles2Km(double valueMiles) {
	return valueMiles*MILE_KM;
}

double m2Ft(double valueMt) {
	return valueMt/FT_M;
}

double Ft2m(double valueFt) {
	return valueFt*FT_M;
}

double m2Nm(double valueMt) {
	return valueMt/NM_M;
}

double Kmh2ms(double valueKmh) {
	return valueKmh/3.6;
}

double ms2Kmh(double valueMs) {
	return valueMs*3.6;
}

double FtMin2ms(double valueFtMin) {
	return valueFtMin*FTMIN_MSEC;
}

double ms2FtMin(double valueMs) {
	return valueMs/FTMIN_MSEC;
}

double calcTotalSpeed(double hSpeed,double vSpeed) {
	return sqrt(pow(hSpeed,2)+pow(vSpeed,2));
}

double latDegMin2rad(int degrees,float minutes,short N) {
	if(N) return (degrees+minutes*SIXTYTH)*DEG2RAD;
	else return -(degrees+minutes*SIXTYTH)*DEG2RAD;
}

double lonDegMin2rad(int degrees,float minutes,short E) {
	if(E) return -(degrees+minutes*SIXTYTH)*DEG2RAD;
	else return (degrees+minutes*SIXTYTH)*DEG2RAD;
}

double latDegMinSec2rad(int deg, int min, float sec, short N) {
	if(N) return (deg+min*SIXTYTH+sec*SEC_HOUR)*DEG2RAD;
	else return -(deg+min*SIXTYTH+sec*SEC_HOUR)*DEG2RAD;
}

double lonDegMinSec2rad(int deg, int min, float sec, short E) {
	if(E) return -(deg+min*SIXTYTH+sec*SEC_HOUR)*DEG2RAD;
	else return (deg+min*SIXTYTH+sec*SEC_HOUR)*DEG2RAD;
}

double absAngle(double angle) { //to put angle in the range between 0 and 2PI
	if(isinf(angle)) return TWO_PI;
	double absAngle=fmod(angle,TWO_PI);
	if(absAngle<0) absAngle+=TWO_PI;
	return absAngle;
}

double calcRhumbLineRoute(double lat1, double lon1, double lat2, double lon2, double *d) { //Loxodrome
	double dlon_W=absAngle(lon2-lon1);
	double dlon_E=absAngle(lon1-lon2);
	double dphi=log(tan(lat2/2+QUARTER_PI)/tan(lat1/2+QUARTER_PI));
	double q,tc;
	if(fabs(lat2-lat1)>SQRT_TOL) q=(lat2-lat1)/dphi;
	else q=cos(lat1);
	if (dlon_W<dlon_E){ // Westerly rhumb line is the shortest
		tc=absAngle(atan2(-dlon_W,dphi));
		*d=sqrt(pow(q,2)*pow(dlon_W,2)+pow(lat2-lat1,2));
	} else {
		tc=absAngle(atan2(dlon_E,dphi));
		*d=sqrt(pow(q,2)*pow(dlon_E,2)+pow(lat2-lat1,2));
	}
	return tc;
}

double calcGreatCircleRoute(double lat1, double lon1, double lat2, double lon2, double *d) { //Ortodrome
	*d=calcAngularDist(lat1,lon1,lat2,lon2);
	if(lat1+lat2==0 && fabs(lon1-lon2)==PI && fabs(lat1)!=HALF_PI) return 0; //Course between antipodal points is undefined!
	double tc;
	if(d==0 || lat1==-HALF_PI) tc=TWO_PI;
	else if (lat1==HALF_PI) tc=PI;
	else if(lon1==lon2) {
		if(lat1>lat2) tc=PI;
		else tc=TWO_PI;
	}
	else if(sin(lon2-lon1)<0) tc=acos((sin(lat2)-sin(lat1)*cos(*d))/(sin(*d)*cos(lat1)));
	else tc=TWO_PI-acos((sin(lat2)-sin(lat1)*cos(*d))/(sin(*d)*cos(lat1)));
	return tc;
}

double calcGreatCircleCourse(double lat1, double lon1, double lat2, double lon2) { //not require pre-computation of distance
	return absAngle(atan2(sin(lon1-lon2)*cos(lat2),cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon1-lon2)));
}

double calcAngularDist(double lat1, double lon1, double lat2, double lon2) {
	return acos(sin(lat1)*sin(lat2)+cos(lat1)*cos(lat2)*cos(lon1-lon2));
}

double calcSmallAngularDist(double lat1, double lon1, double lat2, double lon2) { //less subject to rounding error for short distances
	return 2*asin(sqrt(pow(sin((lat1-lat2)/2),2)+cos(lat1)*cos(lat2)*pow(sin((lon1-lon2)/2),2)));
}

double Rad2Km(double rad) {
	return rad*EARTH_RADIUS_KM;
}

double Km2Rad(double km) {
	return km/EARTH_RADIUS_KM;
}

double Rad2m(double rad) {
	return rad*EARTH_RADIUS_M;
}

double m2Rad(double mt) {
	return mt/EARTH_RADIUS_M;
}

double Rad2Mi(double rad) {
	return rad*EARTH_RADIUS_MI;
}

double Rad2Nm(double rad) {
	return rad*RAD2NM;
}

double Rad2Deg(double rad) {
	return rad*RAD2DEG;
}

double Deg2Rad(double deg) {
	return deg*DEG2RAD;
}

int calcIntermediatePoint(double lat1, double lon1, double lat2, double lon2, double atd, double d, double *latI, double *lonI) {
	if(atd>d) return 0;
	double A=sin(d-atd)/sin(d);
	double B=sin(atd)/sin(d);
	double x=A*cos(lat1)*cos(lon1)+B*cos(lat2)*cos(lon2);
	double y=A*cos(lat1)*sin(lon1)+B*cos(lat2)*sin(lon2);
	double z=A*sin(lat1)+ B*sin(lat2);
	*latI=atan2(z,sqrt(pow(x,2)+pow(y,2)));
	*lonI=atan2(y,x);
	return 1;
}

short isAngleBetween(double low, double angle, double hi) {
	if(low==TWO_PI) low=0;
	if(hi==0) hi=TWO_PI;
	if(low>hi) {
		hi+=TWO_PI;
		if(0<=angle && angle<=PI) angle+=TWO_PI;
	}
	return(low<=angle && angle<=hi);
}

double calcGCCrossTrackError(double lat1, double lon1, double lon2, double latX, double lonX, double course12, double *atd) {
	double dist1X,xtd; //positive XTD means right of course, negative means left
	double course1X=calcGreatCircleRoute(lat1,lon1,latX,lonX,&dist1X);
	if(lat1!=HALF_PI && lat1!=-HALF_PI) xtd=asin(sin(dist1X)*sin(course1X-course12));
	else { //If the point 1 is the N or S Pole replace crs_1X-crs_12 with lonX-lon2 or lon2-lonX, respectively
		double diff;
		if(lat1==HALF_PI) diff=lonX-lon2;
		else diff=lon2-lonX;
		xtd=asin(sin(dist1X)*sin(diff));
	}
	if(isAngleBetween(course12+QUARTER_PI,course1X,course12+PI)) *atd=-acos(cos(dist1X)/cos(xtd));
	else *atd=acos(cos(dist1X)/cos(xtd));
	//For very short distances
  //*atd=asin(sqrt(pow(sin(dist1X),2)-pow(sin(xtd),2))/cos(xtd));
	return xtd;
}

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

void convertDecimal2DegMin(double dec, int *deg, double *min) {
	*deg=(int)floor(dec);
	*min=(dec-*deg)/SIXTYTH;
}

void convertDecimal2DegMinSec(double dec, int *deg, int *min, float *sec) {
	double decimalMin;
	convertDecimal2DegMin(dec,deg,&decimalMin);
	*min=(int)floor(decimalMin);
	*sec=(decimalMin-*min)/SIXTYTH;
}

void convertRad2DegMinSec(double rad, int *deg, int *min, float *sec) {
	convertDecimal2DegMinSec(Rad2Deg(rad),deg,min,sec);
}

void convertTimestamp2HourMinSec(float timestamp, int *hour, int *min, float *sec) {
	*hour=(int)floor(timestamp/3600);
	float temp=timestamp-(*hour)*3600;
	*min=(int)floor(temp/60);
	*sec=temp-(*min)*60;
}

double calcWindDirSpeed(double crs, double hd, double tas, double gs, double *ws) {
	double diffSpeed=tas-gs;
	double diffAngle=hd-crs;
	*ws=sqrt(pow(diffSpeed,2)+4*tas*gs*pow(sin(diffAngle/2),2));
	double wd=crs+atan2(tas*sin(diffAngle),tas*cos(diffAngle)-gs); //assumes atan2(y,x), reverse arguments if your implementation has atan2(x,y) )
	return absAngle(wd);
}

int calcHeadingGroundSpeed(double crs, double wd, double tas, double ws, double *hd, double *gs) {
	double diffAngle=wd-crs;
	double swc=(ws/tas)*sin(diffAngle);
	if(fabs(swc)>1) return -1; //course cannot be flown! wind too strong
	else {
      *hd=crs+asin(swc);
      *hd=absAngle(*hd);
      *gs=tas*sqrt(1-pow(swc,2))-ws*cos(diffAngle);
      if(*gs<0) return 0;  //course cannot be flown-- wind too strong
	}
	return 1;
}

double calcCourseGroundSpeed(double hd, double wd, double tas, double ws, double *gs) {
	double diffAngle=hd-wd;
	*gs=sqrt(pow(ws,2)+pow(tas,2)-2*ws*tas*cos(diffAngle));
	double wca;
	if(ws<tas) wca=asin((ws/(*gs))*sin(diffAngle)); //if the wind correction angle is less than 90 degrees
	else wca=atan2(ws*sin(diffAngle),tas-ws*cos(diffAngle)); //general case formula
	return absAngle(hd+wca);
}

void calcHeadCrossWindComp(double ws, double wd, double rd, double *hw, double *xw) {
	double diffAngle=wd-rd;
	*hw=ws*cos(diffAngle);	//tailwind negative
	*xw=ws*sin(diffAngle);	//positive=wind from right
}

void calcBisector(double currCourse, double nextCourse, double *bisector, double *bisectorOpposite) {
	currCourse=absAngle(currCourse+PI); //to see the direction from current WP
	double diff=currCourse-nextCourse; //angle between the directions
	if(diff>0) { //positive difference: the internal angle in on the right
		diff=absAngle(diff);
		*bisector=absAngle(currCourse-diff/2);
	} else { //negative difference: the internal angle is on the left
		diff=absAngle(-diff);
		*bisector=absAngle(currCourse+diff/2);
	}
	*bisectorOpposite=absAngle(*bisector+PI);
}

short bisectorOverpassed(double currCourse, double nextCourse, double actualCurrCourse) {
	double bisector1, bisector2;
	calcBisector(currCourse,nextCourse,&bisector1,&bisector2);
	if(absAngle(currCourse-bisector1)<PI) //if currCourse is between bisector1 and bisector2 (bisector1 used as 0)
		return isAngleBetween(bisector2,actualCurrCourse,bisector1);
	else //currCourse is between bisector2 and bisector1
		return isAngleBetween(bisector1,actualCurrCourse,bisector2);
}
