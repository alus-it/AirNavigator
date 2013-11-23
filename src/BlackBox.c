//============================================================================
// Name        : BlackBox.c
// Since       : 14/9/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 23/11/2013
// Description : Produces tracelogFiles as XML GPX files
//============================================================================


#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "BlackBox.h"
#include "Common.h"
#include "Configuration.h"
#include "AirCalc.h"

#define MIN_DIST 0.00000109872 // 7 m in Rad

enum blackBoxStatus {
	BBS_NOT_SET,
	BBS_WAIT_FIX,
	BBS_WAIT_POS,
	BBS_WAIT_OPT,
	BBS_PAUSED
};

bool openRecordingFile(void);
void recordPos(double lat, double lon, float timestamp, int hour, int min, float sec);

double lastlat, lastlon, minlat, minlon, maxlat, maxlon;
float currTimestamp,lastTimestamp;
double updateDist; //here in rad
int cyear,cmonth,cday,chour,cmin,csec; //creation time of the track file
char *filename=NULL;
long trackPointCounter;
FILE *tracklogFile=NULL;
enum blackBoxStatus bbStatus=BBS_NOT_SET;

void BlackBoxStart(void) {
	if(bbStatus!=BBS_NOT_SET) return;
	lastTimestamp=-config.recordTimeInterval;
	trackPointCounter=0;
	minlat=90;
	minlon=180;
	maxlat=-90;
	maxlon=-180;
	updateDist=m2Rad(config.recordMinDist);
	if(updateDist<MIN_DIST) updateDist=MIN_DIST;
	struct tm time_str;
	long currTime=time(NULL); //get current time from internal clock
	time_str=*localtime(&currTime); //to obtain day, month, year, hour and minute
	cyear=time_str.tm_year+1900; //year time of creation of the track file
	cmonth=time_str.tm_mon+1; //month
	cday=time_str.tm_mday; //day
	chour=time_str.tm_hour; //hour
	cmin=time_str.tm_min; //minute
	csec=time_str.tm_sec;
	asprintf(&filename,"Track_%d-%02d-%02d_%02d-%02d-%02d.gpx",cyear,cmonth,cday,chour,cmin,csec);
	bbStatus=BBS_WAIT_FIX;
}

bool openRecordingFile(void) {
	char *path;
	asprintf(&path,"%sTracks/%s",BASE_PATH,filename);
	tracklogFile=fopen(path,"w");
	free(path);
	if(tracklogFile==NULL) {
		printLog("BlackBox ERROR: Unable to write track file.\n");
		BlackBoxClose();
		return false;
	}
	fprintf(tracklogFile,"<?xml version=\"1.0\" encoding=\"ISO-8859-1\" standalone=\"yes\"?>\n"
			"<gpx version=\"1.1\" creator=\"AirNavigator by Alus.it\"\n"
			"xmlns=\"http://www.topografix.com/GPX/1/1\"\n"
			"xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n"
			"xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\n"
			"<trk>\n"
			"<name>Track created on %d/%02d/%4d at %2d:%02d:%02d</name>\n"
			"<src>%s - Device serial number ID: %s</src>\n"
			"<trkseg>\n",cday,cmonth,cyear,chour,cmin,csec,config.tomtomModel,config.serialNumber);
	bbStatus=BBS_WAIT_POS;
	return true;
}

void BlackBoxPause(void) {
	if(bbStatus==BBS_WAIT_OPT) BlackBoxCommit();
	bbStatus=BBS_PAUSED;
}

void BlackBoxResume(void) {
	if(bbStatus==BBS_PAUSED && tracklogFile!=NULL) bbStatus=BBS_WAIT_POS;
}

void recordPos(double lat, double lon, float timestamp, int hour, int min, float sec) {
	trackPointCounter++;
	lastlat=lat;
	lastlon=lon;
	lastTimestamp=timestamp;
	lat=Rad2Deg(lat);
	lon=-Rad2Deg(lon); //For the GPX standard East longitudes are positive
	if(lat<minlat) minlat=lat;
	if(lon<minlon) minlon=lon;
	if(lat>maxlat) maxlat=lat;
	if(lon>maxlon) maxlon=lon;
	fprintf(tracklogFile,"<trkpt lat=\"%f\" lon=\"%f\">\n"
			"<time>%d-%02d-%02dT%02d:%02d:%06.3fZ</time>\n",lat,lon,cyear,cmonth,cday,hour,min,sec);
	bbStatus=BBS_WAIT_OPT;
}

bool BlackBoxRecordPos(double lat, double lon, float timestamp, int hour, int min, float sec, short dateChanged) {
	bool retval=false;
	switch(bbStatus) {
		case BBS_WAIT_FIX:
			if(openRecordingFile()) recordPos(lat,lon,timestamp,hour,min,sec);
			break;
		case BBS_WAIT_POS: {
			double deltaT;
			if(dateChanged) lastTimestamp-=86400;
			if(timestamp>lastTimestamp) deltaT=timestamp-lastTimestamp;
			else {
				retval=false;
				break;
			}
			if(deltaT>=config.recordTimeInterval) {
				double deltaS;
				deltaS=calcAngularDist(lat,lon,lastlat,lastlon);
				if(deltaS>=updateDist) {
					recordPos(lat,lon,timestamp,hour,min,sec);
					retval=true;
				}
			}
		} break;
		case BBS_WAIT_OPT:
			retval=BlackBoxCommit();
			break;
		case BBS_NOT_SET: //do nothing
		case BBS_PAUSED: //do nothing
			break;
	}
	return retval;
}

bool BlackBoxRecordAlt(double alt) {
	if(bbStatus!=BBS_WAIT_OPT) return false;
	fprintf(tracklogFile,"<ele>%.1f</ele>\n",alt);
	return true;
}

bool BlackBoxRecordSpeed(double speed) { //Speed in m/s
	if(bbStatus!=BBS_WAIT_OPT) return false;
	fprintf(tracklogFile,"<speed>%.2f</speed>\n",speed);
	return true;
}

bool BlackBoxRecordCourse(double course) {
	if(bbStatus!=BBS_WAIT_OPT) return false;
	fprintf(tracklogFile,"<course>%.2f</course>\n",course);
	return true;
}

bool BlackBoxCommit(void) {
	if(bbStatus!=BBS_WAIT_OPT) return false;
	fprintf(tracklogFile,"</trkpt>\n");
	bbStatus=BBS_WAIT_POS;
	return true;
}

void BlackBoxClose(void) {
	BlackBoxCommit();
	bbStatus=BBS_NOT_SET;
	if(tracklogFile!=NULL) {
		fprintf(tracklogFile,"</trkseg>\n"
				"<number>%ld</number>\n"
				"</trk>\n"
				"<metadata>\n"
				"<name>%s</name>\n"
				"<desc>Track file recorded by AirNavigator</desc>\n"
				"<author>\n"
				"<name>AirNavigator by Alus</name>\n"
				"<email id=\"airnavigator\" domain=\"alus.it\"/>\n"
				"</author>\n"
				"<link href=\"http://www.alus.it/airnavigator\">\n"
				"<text>AirNavigator website</text>\n"
				"</link>\n"
				"<time>%d-%02d-%02dT%02d:%02d:%02dZ</time>\n"
				"<bounds minlat=\"%f\" minlon=\"%f\" maxlat=\"%f\" maxlon=\"%f\"/>\n"
				"</metadata>\n"
				"</gpx>",trackPointCounter,filename,cyear,cmonth,cday,chour,cmin,csec,minlat,minlon,maxlat,maxlon);
		fclose(tracklogFile);
	}
	free(filename);
}

