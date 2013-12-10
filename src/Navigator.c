//============================================================================
// Name        : Navigator.c
// Since       : 19/2/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 10/12/2013
// Description : Navigation manager
//============================================================================


#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <libroxml/roxml.h>
#include "Navigator.h"
#include "Configuration.h"
#include "AirCalc.h"
#include "GPSreceiver.h"
#include "FBrender.h"
#include "HSI.h"
#include "Common.h"


typedef struct wp {
	int seqNo;            //sequence number
	char *name;           //name of the WP
	double latitude;      //rad
	double longitude;     //rad
	double altitude;      //meters
	double dist;          //distance to to this WP in rad
	double initialCourse; //initial true course to to this WP in rad
	double finalCourse;   //final true course to this WP in rad
	double bisector1;     //bisector in rad between this leg and the next
	double bisector2;     //opposite bisector to bisector1 in rad
	double arrTimestamp;  //arrival to this WP timestamp in seconds (from h 0:00)
	struct wp *prev;      //Pointer to the previous waypoint in the list
	struct wp *next;      //Pointer to the next waypoint in the list
} *wayPoint;

struct NavigatorStruct {
	enum navigatorStatus status;
	int numWayPoints;
	double trueCourse, previousAltitude;
	double totalDistKm, prevWPsTotDist; //Km
	double prevWpAvgSpeed, prevTotAvgSpeed; //Km/h
	wayPoint dept, currWP, dest;
	char *routeLogPath;
	FILE *routeLog;
	double sunZenith; //here in rad
};

void NavConfigure(void);
short NavCalculateRoute(void);
void NavFindNextWP(double lat, double lon);
void updateDtgEteEtaAs(double atd, float timestamp, double remainDist);

static struct NavigatorStruct Navigator = {
	.status=NAV_STATUS_NOT_INIT,
	.numWayPoints=0,
	.previousAltitude=-1000,
	.routeLog=NULL
};

void NavConfigure(void) {
	int oldStatus=Navigator.status;
	Navigator.status=NAV_STATUS_NAV_BUSY;
	Navigator.sunZenith=Deg2Rad(config.sunZenith); //rad
	Navigator.prevWpAvgSpeed=config.cruiseSpeed;
	Navigator.prevTotAvgSpeed=config.cruiseSpeed;
	if(oldStatus==NAV_STATUS_NOT_INIT) Navigator.status=NAV_STATUS_NO_ROUTE_SET;
	else Navigator.status=oldStatus;
}

short NavCalculateRoute(void) {
	if(Navigator.status==NAV_STATUS_NO_ROUTE_SET) {
		Navigator.status=NAV_STATUS_NAV_BUSY;
		PrintNavStatus(Navigator.status,"Unknown");
	} else if(Navigator.status!=NAV_STATUS_NAV_BUSY) return -1;
	if(Navigator.dept!=NULL) {
		if(Navigator.numWayPoints>1) {
				if(Navigator.dept->next==NULL) return -3;
		} else if(Navigator.numWayPoints!=1) return -4;
	} else return -5;
	Navigator.totalDistKm=0;
	Navigator.prevWPsTotDist=0;
	Navigator.dest=Navigator.currWP;
	if(Navigator.numWayPoints>1) {
		Navigator.currWP=Navigator.dept->next;
		do {
			Navigator.currWP->initialCourse=calcGreatCircleRoute(Navigator.currWP->prev->latitude,Navigator.currWP->prev->longitude,Navigator.currWP->latitude,Navigator.currWP->longitude,&Navigator.currWP->dist);
			Navigator.currWP->finalCourse=calcGreatCircleFinalCourse(Navigator.currWP->prev->latitude,Navigator.currWP->prev->longitude,Navigator.currWP->latitude,Navigator.currWP->longitude);
			double remainDist=Rad2Km(Navigator.currWP->dist);
			Navigator.totalDistKm+=remainDist;
			fprintf(Navigator.routeLog,"* Travel from %s to %s\n",Navigator.currWP->prev->name,Navigator.currWP->name);
			fprintf(Navigator.routeLog,"Initial course to WP: %07.3f°\n",Rad2Deg(Navigator.currWP->initialCourse));
			fprintf(Navigator.routeLog,"Final   course to WP: %07.3f°\n",Rad2Deg(Navigator.currWP->finalCourse));
			fprintf(Navigator.routeLog,"Horizontal distance to WP is: %.3f Km\n",remainDist);
			if(Navigator.currWP->prev!=Navigator.dept) calcBisector(Navigator.currWP->prev->finalCourse,Navigator.currWP->initialCourse,&Navigator.currWP->prev->bisector1,&Navigator.currWP->prev->bisector2); //calc bisectors for prev WP
			double timeHours=remainDist/config.cruiseSpeed; //hours
			if(Navigator.currWP==Navigator.dept->next) PrintNavRemainingDistWP(remainDist,config.cruiseSpeed,timeHours);
			int hours,mins;
			float secs;
			convertDecimal2DegMinSec(timeHours,&hours,&mins,&secs);
			fprintf(Navigator.routeLog,"Flight time to WP: %2d:%02d:%02d\n",hours,mins,(int)secs);
			fprintf(Navigator.routeLog,"Initial altitude: %.0f m  %.0f Ft\n",Navigator.currWP->prev->altitude,m2Ft(Navigator.currWP->prev->altitude));
			fprintf(Navigator.routeLog,"Final altitude: %.0f m  %.0f Ft\n",Navigator.currWP->altitude,m2Ft(Navigator.currWP->altitude));
			double altDiff=Navigator.currWP->altitude-Navigator.currWP->prev->altitude;
			double slope=altDiff/1000/remainDist;
			fprintf(Navigator.routeLog,"Slope: %.2f %%\n",slope*100);
			double climb=altDiff/(timeHours*3600); // m/s
			fprintf(Navigator.routeLog,"Estimated climb rate: %.2f m/s  %.0f Ft/min\n\n",climb,ms2FtMin(climb)); //just to have an idea
			Navigator.currWP=Navigator.currWP->next;
		} while(Navigator.currWP!=NULL);
		fprintf(Navigator.routeLog,"TOTAL distance from departure along all WPs to Navigator.destination is: %.3f Km\n",Navigator.totalDistKm);
		double totalTimeHours=Navigator.totalDistKm/config.cruiseSpeed;
		int hours,mins;
		float secs;
		convertDecimal2DegMinSec(totalTimeHours,&hours,&mins,&secs);
		fprintf(Navigator.routeLog,"TOTAL flight time: %2d:%02d:%02d\n",hours,mins,(int)secs);
		pthread_mutex_lock(&gps.mutex);
		float timeCalc=gps.timestamp;
		pthread_mutex_unlock(&gps.mutex);
		if(timeCalc<0) timeCalc=getCurrentTime(); //In this case we don't have the time from GPS so we take it from the internal clock
		timeCalc=timeCalc/3600+totalTimeHours; //hours, in order to obtain the ETA
		PrintNavRemainingDistDST(Navigator.totalDistKm,config.cruiseSpeed,timeCalc);
		double fuelNeeded=config.fuelConsumption*totalTimeHours;
		fprintf(Navigator.routeLog,"TOTAL fuel needed: %.2f liters\n\n",fuelNeeded);
		checkDaytime(0);
		Navigator.currWP=Navigator.dept->next;
	} else { //Navigator.numWayPoints==1
		fprintf(Navigator.routeLog,"* Route with only one waypoint: %s\n",Navigator.currWP->name);
		checkDaytime(1);
	}
	if(Navigator.routeLog!=NULL) fclose(Navigator.routeLog); //Close the route log file when not needed
	Navigator.status=NAV_STATUS_TO_START_NAV;
	PrintNavStatus(Navigator.status,Navigator.currWP->name);
	FBrenderFlush();
	return 1;
}

int NavLoadFlightPlan(char* GPXfile) {
	if(GPXfile==NULL) return -1;
	if(Navigator.status==NAV_STATUS_NOT_INIT) NavConfigure();
	if(Navigator.status==NAV_STATUS_NOT_INIT) return -2;
	NavClearRoute();
	Navigator.routeLogPath=strdup(GPXfile);
	int len=strlen(Navigator.routeLogPath);
	Navigator.routeLogPath[len-3]='t';
	Navigator.routeLogPath[len-2]='x';
	Navigator.routeLogPath[len-1]='t';
	Navigator.routeLog=fopen(Navigator.routeLogPath,"w"); //Create the route log file: NavCalculateRoute() will write in it
	if(Navigator.routeLog==NULL) {
		printLog("ERROR not possible to write the route log file.\n");
		Navigator.status=NAV_STATUS_NO_ROUTE_SET;
		return -3;
	}
	node_t* root=roxml_load_doc(GPXfile);
	if(root==NULL) {
		printLog("ERROR no such file '%s'\n",GPXfile);
		return -4;
	}
	char* text=NULL;
	//TODO: here we load just the first route may be there are others routes in the GPX file...
	node_t* route=roxml_get_chld(root,"rte",0);
	if(route==NULL) {
		printLog("ERROR no route found in GPX file: '%s'\n",GPXfile);
		roxml_release(RELEASE_ALL);
		roxml_close(root);
		return -5;
	}
	node_t* wp;
	int total=roxml_get_chld_nb(route);
	if(total<1) {
		printLog("ERROR no waypoints found in route in GPX file: '%s'\n",GPXfile);
		roxml_release(RELEASE_ALL);
		roxml_close(root);
		return 0;
	}
	int wpcounter=0,i;
	node_t* node;
	double latX,lonX,altX;
	for(i=0;i<total;i++) {
		wp=roxml_get_chld(route,NULL,i);
		text=roxml_get_name(wp,NULL,0);
		if(strcmp(text,"rtept")==0) {
			wpcounter++;
			node=roxml_get_attr(wp,"lat",0);
			if(node!=NULL) {
				text=roxml_get_content(node,NULL,0,NULL);
				latX=atof(text);
			} else break; //WP without latitude, skip it
			node=roxml_get_attr(wp,"lon",0);
			if(node!=NULL) {
				text=roxml_get_content(node,NULL,0,NULL);
				lonX=atof(text);
			} else break; //WP without longitude, skip it
			node=roxml_get_chld(wp,"ele",0);
			if(node!=NULL) {
				text=roxml_get_content(node,NULL,0,NULL);
				if(text!=NULL) altX=atof(text);
				else altX=0;
			} else altX=0; //WP altitude missing we put it to 0
			node=roxml_get_chld(wp,"name",0);
			if(node!=NULL) {
				text=roxml_get_content(node,NULL,0,NULL);
				if(text==NULL) asprintf(&text,"Unamed WP %d",wpcounter);
			} else asprintf(&text,"Unamed WP %d",wpcounter);
			NavAddWayPoint(Deg2Rad(latX),Deg2Rad(-lonX),altX,text); //East longitudes are positive according to the GPX standard
		}
	}
	roxml_release(RELEASE_ALL);
	roxml_close(root);
	if(NavCalculateRoute()<0) {
		printLog("ERROR: NavCalculateRoute FAILED.\n");
		NavClearRoute();
		return -3;
	}
	return wpcounter;
}

void NavAddWayPoint(double latWP, double lonWP, double altWPmt, char *WPname) {
	if(Navigator.status!=NAV_STATUS_NO_ROUTE_SET) return;
	wayPoint newWP;
	newWP=(wayPoint)malloc(sizeof(struct wp));
	newWP->name=strdup(WPname);
	newWP->latitude=latWP;
	newWP->longitude=lonWP;
	newWP->altitude=altWPmt;
	newWP->next=NULL;
	if(Navigator.numWayPoints==0) { //its's the first
		Navigator.dept=newWP;
		newWP->prev=NULL;
	} else {
		Navigator.currWP->next=newWP;
		newWP->prev=Navigator.currWP; //link to the previous
	}
	Navigator.currWP=newWP;
	newWP->seqNo=Navigator.numWayPoints++;
}

int NavReverseRoute(void) {
	if(Navigator.status==NAV_STATUS_NAV_BUSY || Navigator.status==NAV_STATUS_NO_ROUTE_SET || Navigator.numWayPoints<2) return 0;
	Navigator.status=NAV_STATUS_NAV_BUSY;
	PrintNavStatus(Navigator.status,"Reversing...");
	Navigator.dest=Navigator.dept;
	Navigator.currWP=Navigator.dept;
	wayPoint next;
	int i=Navigator.numWayPoints-1;
	while(Navigator.currWP->next!=NULL) {
		Navigator.currWP->seqNo=i--;
		next=Navigator.currWP->next;
		Navigator.currWP->next=Navigator.currWP->prev;
		Navigator.currWP->prev=next;
		Navigator.currWP=next;
	}
	Navigator.currWP->seqNo=0;
	Navigator.currWP->next=Navigator.currWP->prev;
	Navigator.currWP->prev=NULL;
	Navigator.dept=Navigator.currWP;
	Navigator.currWP=Navigator.dest;
	Navigator.routeLog=fopen(Navigator.routeLogPath,"a+"); //Reopen the route log file to write about the reversed route
	if(Navigator.routeLog==NULL) {
		printLog("ERROR not possible to write the route log file.\n");
		NavClearRoute();
		return 0;
	}
	fprintf(Navigator.routeLog,"\n\n\nREVERSED ROUTE\n\n");
	if(NavCalculateRoute()<0) {
		printLog("ERROR: NavCalculateRoute FAILED!\n");
		NavClearRoute();
		return 0;
	}
	return 1;
}

void NavClearRoute(void) {
	Navigator.status=NAV_STATUS_NAV_BUSY;
	PrintNavStatus(Navigator.status,"Unknown");
	if(Navigator.numWayPoints!=0) {
		do {
			Navigator.currWP=Navigator.dept;
			free(Navigator.currWP->name);
			Navigator.dept=Navigator.currWP->next;
			free(Navigator.currWP);
		} while(Navigator.dept!=NULL);
		Navigator.numWayPoints=0;
	}
	free(Navigator.routeLogPath);
	Navigator.routeLogPath=NULL;
	PrintNavStatus(Navigator.status,"Nowhere");
	Navigator.status=NAV_STATUS_NO_ROUTE_SET;
}

void NavClose(void) {
	NavClearRoute();
	Navigator.status=NAV_STATUS_NOT_INIT;
}

void NavFindNextWP(double lat, double lon) {
	if(Navigator.status!=NAV_STATUS_TO_START_NAV && Navigator.status!=NAV_STATUS_WAIT_FIX) return;
	if(Navigator.numWayPoints==1) {
		Navigator.currWP=Navigator.dest;
		Navigator.status=NAV_STATUS_NAV_TO_SINGLE_WP;
		return;
	}
	wayPoint i, nearest=Navigator.dept;
	double shortestDist=calcAngularDist(lat,lon,Navigator.dept->latitude,Navigator.dept->longitude);
	for(i=Navigator.dept->next;i!=NULL;i=i->next) { //searching the nearest
		double distance=calcAngularDist(lat,lon,i->latitude,i->longitude);
		if(distance<=shortestDist) {
			shortestDist=distance;
			nearest=i;
		}
	}
	printLog("\nNearest waypoint is: %s No: %d at %f Km\n",nearest->name,nearest->seqNo,Rad2Km(shortestDist));
	if(nearest==Navigator.dest) {
		if(Navigator.dept->latitude==Navigator.dest->latitude && Navigator.dept->longitude==Navigator.dest->longitude) { //Navigator.dept and Navigator.dest are the same place
			if(calcAngularDist(lat,lon,Navigator.dept->latitude,Navigator.dept->longitude)<m2Rad(config.deptDistTolerance)) Navigator.currWP=Navigator.dept->next;
			else Navigator.currWP=Navigator.dest;
		} else Navigator.currWP=Navigator.dest;
	} else { //The nearest WP isn't the Navigator.destination
		if(calcAngularDist(lat,lon,nearest->next->latitude,nearest->next->longitude)<=nearest->next->dist) Navigator.currWP=nearest->next;
		else Navigator.currWP=nearest;
	}
	if(Navigator.currWP==Navigator.dept) {
		if(calcAngularDist(lat,lon,Navigator.dept->latitude,Navigator.dept->longitude)<m2Rad(config.deptDistTolerance)) { //we are near the dpt
			Navigator.currWP=Navigator.dept->next;
			Navigator.status=NAV_STATUS_NAV_TO_WPT;
		} else Navigator.status=NAV_STATUS_NAV_TO_DPT; //we have still to go to the departure
	} else { //we are not traveling to the departure
		if(Navigator.currWP->seqNo>1) { //  //Navigator.currWP->prev!=Navigator.dept  if we aren't traveling direcly from the departure
			Navigator.prevWPsTotDist=0;
			for(i=Navigator.dept->next;i!=Navigator.currWP;i=i->next) Navigator.prevWPsTotDist+=i->dist;
			Navigator.prevWPsTotDist=Rad2Km(Navigator.prevWPsTotDist);
			Navigator.currWP->prev->arrTimestamp=Navigator.dept->arrTimestamp+(Navigator.prevWPsTotDist/config.cruiseSpeed)*3600; //ETA to the previous WP, adding the time to reach the previous WP
		}
		Navigator.status=NAV_STATUS_NAV_TO_WPT;
	}

	if(Navigator.currWP==Navigator.dest) Navigator.status=NAV_STATUS_NAV_TO_DST;
	PrintNavStatus(Navigator.status,Navigator.currWP->name);
	printLog("Next waypoint is: %s\n\n\n",Navigator.currWP->name);
	FBrenderFlush();
}

void NavStartNavigation() {
	if(Navigator.status!=NAV_STATUS_TO_START_NAV) return;
	pthread_mutex_lock(&gps.mutex);
	float timestamp=gps.timestamp; //timestamp is the real time when we start the travel try to get it from GPS
	if(timestamp==-1) timestamp=getCurrentTime(); //if not valid get it from internal clock
	if(timestamp<0) {
		pthread_mutex_unlock(&gps.mutex);
		return;
	}
	if(gps.fixMode>MODE_NO_FIX && gps.lat!=100) { //if have fix give immediately the position to the nav. The lat!=100 is just to avoid the case of having fix but still not a position stored
		NavFindNextWP(gps.lat,gps.lon);
		if(Navigator.status==NAV_STATUS_NAV_TO_WPT || Navigator.status==NAV_STATUS_NAV_TO_DST || Navigator.status==NAV_STATUS_NAV_TO_SINGLE_WP) Navigator.dept->arrTimestamp=timestamp; //record the starting time for whole route
		NavUpdatePosition(gps.lat,gps.lon,gps.realAltMt,gps.speedKmh,gps.trueTrack,timestamp);
	} else {
		if(Navigator.numWayPoints>1) Navigator.currWP=Navigator.dept->next;
		else Navigator.currWP=Navigator.dest;
		Navigator.status=NAV_STATUS_WAIT_FIX;
	}
	pthread_mutex_unlock(&gps.mutex);
	PrintNavStatus(Navigator.status,Navigator.currWP->name);
	FBrenderFlush();
}

void updateDtgEteEtaAs(double atd, float timestamp, double remainDist) {
	double averageSpeed; //Km/h
	double timeVal; //Hours
	if(timestamp>Navigator.currWP->prev->arrTimestamp) { //to avoid infinite, null or negative speed and time
		remainDist=Rad2Km(remainDist); //Km
		if(atd>=0) {
			averageSpeed=ms2Kmh(Rad2m(atd)/(timestamp-Navigator.currWP->prev->arrTimestamp));
			Navigator.prevWpAvgSpeed=averageSpeed;
		} else averageSpeed=Navigator.prevWpAvgSpeed; //with negative ATDs we estimate using previous average speed
		timeVal=remainDist/averageSpeed; //ETE (remaining time) in hours
		PrintNavRemainingDistWP(remainDist,averageSpeed,timeVal);
	}
	if(timestamp>Navigator.dept->arrTimestamp) {
		double totCoveredDistKm=Navigator.prevWPsTotDist+Rad2Km(atd);
		if(atd>=0) {
			averageSpeed=ms2Kmh((totCoveredDistKm*1000)/(timestamp-Navigator.dept->arrTimestamp)); //Km/h
			Navigator.prevTotAvgSpeed=averageSpeed;
		} else averageSpeed=Navigator.prevTotAvgSpeed;
		remainDist=Navigator.totalDistKm-totCoveredDistKm; //Km
		timeVal=remainDist/averageSpeed; //hours
		timeVal+=timestamp/3600; //hours, in order to obtain the ETA
		PrintNavRemainingDistDST(remainDist,averageSpeed,timeVal);
	}
}

void NavUpdatePosition(double lat, double lon, double altMt, double speedKmh, double dir, float timestamp) {
	double actualTrueCourse, remainDist;
	switch(Navigator.status) {
		case NAV_STATUS_NOT_INIT:     //Do nothing...
		case NAV_STATUS_NO_ROUTE_SET:
			break;
		case NAV_STATUS_TO_START_NAV:
			if(Navigator.previousAltitude==-1000) Navigator.previousAltitude=altMt;
			else if(altMt-Navigator.previousAltitude>config.takeOffdiffAlt && speedKmh>config.stallSpeed) { //in this case start the navigation
				NavFindNextWP(lat,lon);
				if(Navigator.status==NAV_STATUS_NAV_TO_WPT || Navigator.status==NAV_STATUS_NAV_TO_DST || Navigator.status==NAV_STATUS_NAV_TO_SINGLE_WP) Navigator.dept->arrTimestamp=timestamp; //record the starting time for whole route
				NavUpdatePosition(lat,lon,altMt,speedKmh,dir,timestamp); //recursive call
				PrintNavStatus(Navigator.status,Navigator.currWP->name);
				break;
			}
			if(Navigator.numWayPoints>1) actualTrueCourse=calcGreatCircleRoute(lat,lon,Navigator.dept->next->latitude,Navigator.dept->next->longitude,&remainDist); //calc just course and distance
			else actualTrueCourse=calcGreatCircleRoute(lat,lon,Navigator.dest->latitude,Navigator.dest->longitude,&remainDist); //numWayPoint==1
			PrintNavDTG(remainDist);
			HSIupdateCDI(Rad2Deg(actualTrueCourse),0);
			break;
		case NAV_STATUS_NAV_TO_DPT: //we are still going to the departure point
			actualTrueCourse=calcGreatCircleRoute(lat,lon,Navigator.dept->latitude,Navigator.dept->longitude,&remainDist); //calc just course and distance
			PrintNavDTG(remainDist);
			HSIupdateCDI(Rad2Deg(actualTrueCourse),0);
			if(remainDist<m2Rad(config.deptDistTolerance)) {
				Navigator.currWP=Navigator.dept->next;
				Navigator.dept->arrTimestamp=timestamp; //here we record the starting time for whole route
				if(Navigator.currWP!=Navigator.dest) Navigator.status=NAV_STATUS_NAV_TO_WPT;
				else Navigator.status=NAV_STATUS_NAV_TO_DST; //Next WP is already the final Navigator.destination
				PrintNavStatus(Navigator.status,Navigator.currWP->name);
				NavUpdatePosition(lat,lon,altMt,speedKmh,dir,timestamp); //recursive call
			}
			break;
		case NAV_STATUS_NAV_TO_WPT: {
			double atd, trackErr;
			trackErr=Rad2m(calcGCCrossTrackError(Navigator.currWP->prev->latitude,Navigator.currWP->prev->longitude,Navigator.currWP->longitude,lat,lon,Navigator.currWP->initialCourse,&atd));
			if(atd>=0) remainDist=Navigator.currWP->dist-atd;
			else remainDist=Navigator.currWP->dist+fabs(atd); //negative ATD: we are still before the prev WP
			actualTrueCourse=calcGreatCircleCourse(lat,lon,Navigator.currWP->latitude,Navigator.currWP->longitude); //Find the direct direction to curr WP (needed to check if we passed the bisector)
			if(atd>=0 && (atd>=Navigator.currWP->dist || bisectorOverpassed(Navigator.currWP->finalCourse,actualTrueCourse,Navigator.currWP->bisector1,Navigator.currWP->bisector2))) { //consider this WP as reached
				Navigator.currWP->arrTimestamp=timestamp;
				Navigator.prevWPsTotDist+=Rad2Km(Navigator.currWP->dist);
				Navigator.currWP=Navigator.currWP->next;
				if(Navigator.currWP==Navigator.dest) Navigator.status=NAV_STATUS_NAV_TO_DST; //Next WP is the final Navigator.destination
				PrintNavStatus(Navigator.status,Navigator.currWP->name);
				NavUpdatePosition(lat,lon,altMt,speedKmh,dir,timestamp); //Recursive call on the new WayPoint
				return;
			} //else the WP or bisector is still not reached...
			if(fabs(trackErr)>config.trackErrorTolearnce) { //if we have bigger error
				double latI,lonI; //the perpendicular point on the route
				calcIntermediatePoint(Navigator.currWP->prev->latitude,Navigator.currWP->prev->longitude,Navigator.currWP->latitude,Navigator.currWP->longitude,atd,Navigator.currWP->dist,&latI,&lonI);
				actualTrueCourse=calcGreatCircleCourse(latI,lonI,Navigator.currWP->latitude,Navigator.currWP->longitude);
			} //else with a small error the actualTrueCourse calculated before is fine
			HSIupdateCDI(Rad2Deg(actualTrueCourse),trackErr);
			PrintNavTrackATD(atd);
			if(atd>=0) HSIupdateVSI(m2Ft((Navigator.currWP->altitude-Navigator.currWP->prev->altitude)/Navigator.currWP->dist*atd+Navigator.currWP->prev->altitude));
			updateDtgEteEtaAs(atd,timestamp,remainDist);
		} break;
		case NAV_STATUS_NAV_TO_DST: {
			double atd, trackErr;
			trackErr=Rad2m(calcGCCrossTrackError(Navigator.currWP->prev->latitude,Navigator.currWP->prev->longitude,Navigator.currWP->longitude,lat,lon,Navigator.currWP->initialCourse,&atd));
			if(atd>=0) remainDist=Navigator.currWP->dist-atd;
			else remainDist=Navigator.currWP->dist+fabs(atd); //negative ATD: we are still before the prev WP
			if(atd>=Navigator.currWP->dist) { //consider Navigator.destination as reached (90 degrees bisector)
				Navigator.currWP->arrTimestamp=timestamp;
				Navigator.status=NAV_STATUS_END_NAV;
				PrintNavStatus(Navigator.status,"Nowhere");
				NavUpdatePosition(lat,lon,altMt,speedKmh,dir,timestamp); //Recursive call on the new WayPoint
				return;
			} //else the Navigator.destination is still not reached...
			if(fabs(trackErr)<config.trackErrorTolearnce) //with a really small error
				actualTrueCourse=calcGreatCircleCourse(lat,lon,Navigator.currWP->latitude,Navigator.currWP->longitude); //just find the direct direction to the Navigator.destination
			else { //otherwise we have bigger error and so we calculate it better...
				double latI,lonI; //the perpendicular point on the route
				calcIntermediatePoint(Navigator.currWP->prev->latitude,Navigator.currWP->prev->longitude,Navigator.currWP->latitude,Navigator.currWP->longitude,atd,Navigator.currWP->dist,&latI,&lonI);
				actualTrueCourse=calcGreatCircleCourse(latI,lonI,Navigator.currWP->latitude,Navigator.currWP->longitude);
			}
			PrintNavTrackATD(atd);
			HSIupdateCDI(Rad2Deg(actualTrueCourse),trackErr);
			if(atd>=0) HSIupdateVSI(m2Ft((Navigator.currWP->altitude-Navigator.currWP->prev->altitude)/Navigator.currWP->dist*atd+Navigator.currWP->prev->altitude));
			updateDtgEteEtaAs(atd,timestamp,remainDist);
		} break;
		case NAV_STATUS_NAV_TO_SINGLE_WP: {
			actualTrueCourse=calcGreatCircleRoute(lat,lon,Navigator.dest->latitude,Navigator.dest->longitude,&remainDist); //calc just course and distance
			PrintNavDTG(remainDist);
			HSIupdateCDI(Rad2Deg(actualTrueCourse),0);
			double timeVal; //Hours
			remainDist=Rad2Km(remainDist); //Km
			timeVal=remainDist/speedKmh; //ETE (remaining time) in hours
			PrintNavRemainingDistWP(remainDist,-1,timeVal);
			timeVal+=timestamp/3600; //hours, in order to obtain the ETA
			PrintNavRemainingDistDST(remainDist,-1,timeVal);
		} break;
		case NAV_STATUS_END_NAV: //We have reached or passed the Navigator.destination
			actualTrueCourse=calcGreatCircleRoute(lat,lon,Navigator.dest->latitude,Navigator.dest->longitude,&remainDist); //calc just course and distance
			PrintNavDTG(remainDist);
			HSIupdateCDI(Rad2Deg(actualTrueCourse),0);
			break;
		case NAV_STATUS_NAV_BUSY: //Do nothing
			break;
		case NAV_STATUS_WAIT_FIX:
			NavFindNextWP(lat,lon);
			NavUpdatePosition(lat,lon,altMt,speedKmh,dir,timestamp);
			break;
		default: //unknown state, we should be never here
			break;
	}
}

void NavSkipCurrentWayPoint(void) {
	if(Navigator.status==NAV_STATUS_NAV_TO_WPT || Navigator.status==NAV_STATUS_NAV_TO_DST) {
		Navigator.status=NAV_STATUS_NAV_BUSY;
		PrintNavStatus(Navigator.status,"Skipping next WP");
		if(Navigator.currWP==Navigator.dest) {
			Navigator.status=NAV_STATUS_END_NAV;
			PrintNavStatus(Navigator.status,"Nowhere");
			return;
		}
		pthread_mutex_lock(&gps.mutex);
		double lat=gps.lat;
		double lon=gps.lon;
		float timestamp=gps.timestamp;
		pthread_mutex_unlock(&gps.mutex);
		Navigator.currWP->arrTimestamp=timestamp; //we put the arrival timestamp when we skip it
		double atd;
		calcGCCrossTrackError(Navigator.currWP->prev->latitude,Navigator.currWP->prev->longitude,Navigator.currWP->longitude,lat,lon,Navigator.currWP->initialCourse,&atd);
		Navigator.prevWPsTotDist+=Rad2Km(atd); //we add only the ATD
		Navigator.totalDistKm-=Rad2Km(Navigator.currWP->dist)-Rad2Km(atd); //from the total substract the skipped reamaining dst
		Navigator.totalDistKm-=Navigator.currWP->next->dist; //from the total substract the dst from skipped wp to the next
		Navigator.currWP=Navigator.currWP->next; //jump to the next
		Navigator.currWP->initialCourse=calcGreatCircleRoute(lat,lon,Navigator.currWP->latitude,Navigator.currWP->longitude,&Navigator.currWP->dist); //recalc course
		Navigator.totalDistKm+=Rad2Km(Navigator.currWP->dist); //we add the new dst form current pos to the next WP
		if(Navigator.currWP!=Navigator.dest) Navigator.status=NAV_STATUS_NAV_TO_WPT;
		else Navigator.status=NAV_STATUS_NAV_TO_DST;
		PrintNavStatus(Navigator.status,Navigator.currWP->name);
	}
}

short checkDaytime(short calcOnlyDest) {
	short retVal=1;
	double riseTime,setTime;
	struct tm time_str;
	long timestamp=time(NULL); //get current time
	time_str=*localtime(&timestamp); //to obtain day, month, year, hour and minute
	int year=time_str.tm_year+1900; //year
	int month=time_str.tm_mon+1; //month
	int day=time_str.tm_mday; //day
	int hour=time_str.tm_hour; //hour
	int min=time_str.tm_min; //minute
	double arrTimeDecimal=-1;
	int rHour,rMin,sHour,sMin;
	float rSec,sSec;
	if(!calcOnlyDest) {
		fprintf(Navigator.routeLog,"Check daytime on: %d/%02d/%d %2d:%02d\n",day,month,year,hour,min);
		calcSunriseSunset(Navigator.dept->latitude,Navigator.dept->longitude,day,month,year,Navigator.sunZenith,+1,&riseTime,&setTime);
		if(riseTime==-1){
			fprintf(Navigator.routeLog,"WARNING: The sun never rises on the departure location (on the specified date).\n");
			retVal=0;
		}
		double depTimeDecimal=hour+min*SIXTYTH;
		if(depTimeDecimal<riseTime || depTimeDecimal>setTime) {
			fprintf(Navigator.routeLog,"WARNING: Departure at this time will be in the night.\n");
			retVal=0;
		}
		convertDecimal2DegMinSec(riseTime,&rHour,&rMin,&rSec);
		convertDecimal2DegMinSec(setTime,&sHour,&sMin,&sSec);
		fprintf(Navigator.routeLog,"Departure location Sunrise: %2d:%02d, Sunset: %2d:%02d \n",rHour,rMin,sHour,sMin);
		arrTimeDecimal=depTimeDecimal+Navigator.totalDistKm/config.cruiseSpeed;
		if(arrTimeDecimal>24) {
			timestamp+=arrTimeDecimal*3600;
			time_str=*localtime(&timestamp); //to obtain day, month, year, hour and minute
			year=time_str.tm_year+1900; //year
			month=time_str.tm_mon+1; //month
			day=time_str.tm_mday; //day
		}
	}
	calcSunriseSunset(Navigator.dest->latitude,Navigator.dest->longitude,day,month,year,Navigator.sunZenith,+1,&riseTime,&setTime);
	if(riseTime==-1){
		fprintf(Navigator.routeLog,"WARNING: The sun never rises on the arrival location (on the specified date).\n");
		retVal=0;
	}
	if(!calcOnlyDest) {
		if(arrTimeDecimal<riseTime || arrTimeDecimal>setTime) {
			fprintf(Navigator.routeLog,"WARNING: Arrival at this time will be in the night.\n");
			retVal=0;
		} else retVal=-1; //we don't know the estimated time of arrival yet if we are working only with the Navigator.destination
	}
	convertDecimal2DegMinSec(riseTime,&rHour,&rMin,&rSec);
	convertDecimal2DegMinSec(setTime,&sHour,&sMin,&sSec);
	fprintf(Navigator.routeLog,"Arrival location Sunrise: %2d:%02d, Sunset: %2d:%02d \n\n\n",rHour,rMin,sHour,sMin);
	return retVal;
}

enum navigatorStatus NavGetStatus(void) {
	return Navigator.status;
}
