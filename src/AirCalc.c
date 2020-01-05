//============================================================================
// Name        : AirCalc.c
// Since       : 19/2/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : https://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2016 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/alus-it/AirNavigator.git
// Last change : 28/2/2016
// Description : Collection of functions for air navigation calculation
//============================================================================

//Useful functions:  Verify not getting NaN:      if(isnan(number))
//                   Verify not getting infinity: if(isinf(number))

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
#define SEC_HOUR        0.00027777777777777778  // 1/3600
#define DEG2RAD         (M_PI/180)
#define RAD2DEG         (180/M_PI)
#define RAD2NM          ((180*60)/M_PI)

//TOL is a small number of order machine precision- say 1e-15 (here 1e-16).
//this is used in calcRhumbLineRoute to avoid 0/0 indeterminacies on E-W courses.
#define SQRT_TOL 1e-8 //sqrt(TOL)=sqrt(1e-16)=1e-8 (6.4 cm)


double Km2Nm(const double valueKmh) {
	return valueKmh/NM_KM;
}

double Nm2Km(const double valueNm) {
	return valueNm*NM_KM;
}

double Km2Miles(const double valueKm) {
	return valueKm/MILE_KM;
}

double Miles2Km(const double valueMiles) {
	return valueMiles*MILE_KM;
}

double m2Ft(const double valueMt) {
	return valueMt/FT_M;
}

double Ft2m(const double valueFt) {
	return valueFt*FT_M;
}

double m2Nm(const double valueMt) {
	return valueMt/NM_M;
}

double Kmh2ms(const double valueKmh) {
	return valueKmh/3.6;
}

double ms2Kmh(const double valueMs) {
	return valueMs*3.6;
}

double FtMin2ms(const double valueFtMin) {
	return valueFtMin*FTMIN_MSEC;
}

double ms2FtMin(double valueMs) {
	return valueMs/FTMIN_MSEC;
}

double calcTotalSpeed(const double hSpeed, const double vSpeed) {
	return sqrt(pow(hSpeed,2)+pow(vSpeed,2));
}

double latDegMin2rad(const int degrees, const float minutes, const bool N) {
	if(N) return (degrees+minutes*SIXTYTH)*DEG2RAD; //North latitudes positive
	else return -(degrees+minutes*SIXTYTH)*DEG2RAD; //South latitudes negative
}

double lonDegMin2rad(const int degrees, const float minutes, const bool E) {
	if(E) return -(degrees+minutes*SIXTYTH)*DEG2RAD; //Here we consider East longitudes as negative
	else return (degrees+minutes*SIXTYTH)*DEG2RAD; //... and West longitudes as positive
}

double latDegMinSec2rad(const int deg, const int min, const float sec, const bool N) {
	if(N) return (deg+min*SIXTYTH+sec*SEC_HOUR)*DEG2RAD;
	else return -(deg+min*SIXTYTH+sec*SEC_HOUR)*DEG2RAD;
}

double lonDegMinSec2rad(const int deg, const int min, const float sec, const bool E) {
	if(E) return -(deg+min*SIXTYTH+sec*SEC_HOUR)*DEG2RAD;
	else return (deg+min*SIXTYTH+sec*SEC_HOUR)*DEG2RAD;
}

double absAngle(const double angle) { //to put angle in the range between 0 and 2PI
	if(isinf(angle)) return TWO_PI;
	double absangle=fmod(angle,TWO_PI);
	if(absangle<0) absangle+=TWO_PI;
	return absangle;
}

//double calcRhumbLineRoute(const double lat1, const double lon1, const double lat2, const double lon2, double *d) { //Loxodrome: not used
//	double dlon_W=absAngle(lon2-lon1);
//	double dlon_E=absAngle(lon1-lon2);
//	double dphi=log(tan(lat2/2+M_PI_4)/tan(lat1/2+M_PI_4));
//	double q,tc;
//	if(fabs(lat2-lat1)>SQRT_TOL) q=(lat2-lat1)/dphi;
//	else q=cos(lat1);
//	if (dlon_W<dlon_E){ // Westerly rhumb line is the shortest
//		tc=absAngle(atan2(-dlon_W,dphi));
//		*d=sqrt(pow(q,2)*pow(dlon_W,2)+pow(lat2-lat1,2));
//	} else {
//		tc=absAngle(atan2(dlon_E,dphi));
//		*d=sqrt(pow(q,2)*pow(dlon_E,2)+pow(lat2-lat1,2));
//	}
//	return tc;
//}

double calcGreatCircleRoute(const double lat1, const double lon1, const double lat2, const double lon2, double *d) { //Ortodrome
	*d=calcAngularDist(lat1,lon1,lat2,lon2);
	if(lat1+lat2==0 && fabs(lon1-lon2)==M_PI && fabs(lat1)!=M_PI_2) return 0; //Course between antipodal points is undefined!
	if(d==0 || lat1==-M_PI_2) return TWO_PI; // distance null or starting from S pole
	if (lat1==M_PI_2) return M_PI; //starting from N pole
	if(lon1==lon2) {
		if(lat1>lat2) return M_PI;
		else return TWO_PI;
	}
	double tc=acos((sin(lat2)-sin(lat1)*cos(*d))/(sin(*d)*cos(lat1)));
	if(sin(lon2-lon1)>0) tc=TWO_PI-tc;
	return tc;
}

double calcGreatCircleCourse(const double lat1, const double lon1, const double lat2, const double lon2) { //not require pre-computation of distance
	if(lat2==M_PI_2) return TWO_PI; //we are going to N pole
	if(lat2==-M_PI_2) return M_PI; //we are going to S pole
	if(lon1==lon2) {
			if(lat1>lat2) return M_PI;
			else return TWO_PI;
	}
	return absAngle(atan2(sin(lon1-lon2)*cos(lat2),cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon1-lon2)));
}

double calcGreatCircleFinalCourse(const double lat1, const double lon1, const double lat2, const double lon2) {
	return absAngle(calcGreatCircleCourse(lat2,lon2,lat1,lon1)+M_PI);
}

double calcAngularDist(const double lat1, const double lon1, const double lat2, const double lon2) {
	return acos(sin(lat1)*sin(lat2)+cos(lat1)*cos(lat2)*cos(lon1-lon2));
}

double calcSmallAngularDist(const double lat1, const double lon1, const double lat2, const double lon2) { //less subject to rounding error for short distances
	return 2*asin(sqrt(pow(sin((lat1-lat2)/2),2)+cos(lat1)*cos(lat2)*pow(sin((lon1-lon2)/2),2)));
}

double Rad2Km(const double rad) {
	return rad*EARTH_RADIUS_KM;
}

double Km2Rad(const double km) {
	return km/EARTH_RADIUS_KM;
}

double Rad2m(const double rad) {
	return rad*EARTH_RADIUS_M;
}

double m2Rad(const double mt) {
	return mt/EARTH_RADIUS_M;
}

double Rad2Mi(const double rad) {
	return rad*EARTH_RADIUS_MI;
}

double Rad2Nm(const double rad) {
	return rad*RAD2NM;
}

double Rad2Deg(const double rad) {
	return rad*RAD2DEG;
}

double Deg2Rad(const double deg) {
	return deg*DEG2RAD;
}

bool calcIntermediatePoint(const double lat1, const double lon1, const double lat2, const double lon2, const double atd, const double d, double *latI, double *lonI) {
	if(atd>d) return false;
	double A=sin(d-atd)/sin(d);
	double B=sin(atd)/sin(d);
	double x=A*cos(lat1)*cos(lon1)+B*cos(lat2)*cos(lon2);
	double y=A*cos(lat1)*sin(lon1)+B*cos(lat2)*sin(lon2);
	double z=A*sin(lat1)+ B*sin(lat2);
	*latI=atan2(z,sqrt(pow(x,2)+pow(y,2)));
	*lonI=atan2(y,x);
	return true;
}

bool isAngleBetween(double low, double angle, double hi) {
	if(low==TWO_PI) low=0;
	if(hi==0) hi=TWO_PI;
	if(low>hi) {
		hi+=TWO_PI;
		if(0<=angle && angle<=M_PI) angle+=TWO_PI;
	}
	return(low<=angle && angle<=hi);
}

double calcGCCrossTrackError(const double lat1, const double lon1, const double lon2, const double latX, const double lonX, const double course12, double *atd) {
	double dist1X,xtd; //positive XTD means right of course, negative means left
	double course1X=calcGreatCircleRoute(lat1,lon1,latX,lonX,&dist1X);
	if(lat1!=M_PI_2 && lat1!=-M_PI_2) xtd=asin(sin(dist1X)*sin(course1X-course12));
	else { //If the point 1 is the N or S Pole replace crs_1X-crs_12 with lonX-lon2 or lon2-lonX, respectively
		double diff;
		if(lat1==M_PI_2) diff=lonX-lon2;
		else diff=lon2-lonX;
		xtd=asin(sin(dist1X)*sin(diff));
	}
	if(isAngleBetween(course12+M_PI_4,course1X,course12+M_PI)) *atd=-acos(cos(dist1X)/cos(xtd));
	else *atd=acos(cos(dist1X)/cos(xtd)); //to have cos(xtd)=0 xtd=PI so we are terribly out of track, in this case doesn't matter to have a NaN
	//For very short distances
	// *atd=asin(sqrt(pow(sin(dist1X),2)-pow(sin(xtd),2))/cos(xtd));
	return xtd;
}

void convertDecimal2DegMin(const double dec, int *deg, double *min) {
	*deg=(int)floor(dec);
	*min=(dec-*deg)/SIXTYTH;
}

void convertDecimal2DegMinSec(const double dec, int *deg, int *min, float *sec) {
	double decimalMin;
	convertDecimal2DegMin(dec,deg,&decimalMin);
	*min=(int)floor(decimalMin);
	*sec=(decimalMin-*min)/SIXTYTH;
}

void convertRad2DegMinSec(const double rad, int *deg, int *min, float *sec) {
	convertDecimal2DegMinSec(Rad2Deg(rad),deg,min,sec);
}

void convertTimestamp2HourMinSec(const float timestamp, int *hour, int *min, float *sec) {
	*hour=(int)floor(timestamp/3600);
	float temp=timestamp-(*hour)*3600;
	*min=(int)floor(temp/60);
	*sec=temp-(*min)*60;
}

double calcWindDirSpeed(const double crs, const double hd, const double tas, const double gs, double *ws) {
	double diffSpeed=tas-gs;
	double diffAngle=hd-crs;
	*ws=sqrt(pow(diffSpeed,2)+4*tas*gs*pow(sin(diffAngle/2),2));
	double wd=crs+atan2(tas*sin(diffAngle),tas*cos(diffAngle)-gs); //assumes atan2(y,x), reverse arguments if your implementation has atan2(x,y) )
	return absAngle(wd);
}

int calcHeadingGroundSpeed(const double crs, const double wd, const double tas, const double ws, double *hd, double *gs) {
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

double calcCourseGroundSpeed(const double hd, const double wd, const double tas, const double ws, double *gs) {
	double diffAngle=hd-wd;
	*gs=sqrt(pow(ws,2)+pow(tas,2)-2*ws*tas*cos(diffAngle));
	double wca;
	if(ws<tas) wca=asin((ws/(*gs))*sin(diffAngle)); //if the wind correction angle is less than 90 degrees
	else wca=atan2(ws*sin(diffAngle),tas-ws*cos(diffAngle)); //general case formula
	return absAngle(hd+wca);
}

void calcHeadCrossWindComp(const double ws, const double wd, const double rd, double *hw, double *xw) {
	double diffAngle=wd-rd;
	*hw=ws*cos(diffAngle);	//tailwind negative
	*xw=ws*sin(diffAngle);	//positive=wind from right
}

void calcBisector(double currCourse, const double nextCourse, double *bisector, double *bisectorOpposite) {
	currCourse=absAngle(currCourse+M_PI); //to see the direction from current WP
	double diff=currCourse-nextCourse; //angle between the directions
	if(diff>0) { //positive difference: the internal angle in on the right
		diff=absAngle(diff);
		*bisector=absAngle(currCourse-diff/2);
	} else { //negative difference: the internal angle is on the left
		diff=absAngle(-diff);
		*bisector=absAngle(currCourse+diff/2);
	}
	*bisectorOpposite=absAngle(*bisector+M_PI);
}

bool bisectorOverpassed(const double currCourse, const double actualCurrCourse, const double bisector1, const double bisector2) {
	if(absAngle(currCourse-bisector1)<M_PI) //if currCourse is between bisector1 and bisector2 (bisector1 used as 0)
		return isAngleBetween(bisector2,actualCurrCourse,bisector1);
	else //currCourse is between bisector2 and bisector1
		return isAngleBetween(bisector1,actualCurrCourse,bisector2);
}
