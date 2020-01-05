//============================================================================
// Name        : BlackBox.c
// Since       : 14/9/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : https://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2020 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/alus-it/AirNavigator.git
// Last change : 28/2/2016
// Description : Produces tracelogFiles as XML GPX files
//============================================================================


#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "BlackBox.h"
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

struct BlackBoxStruct {
	double lastlat, lastlon, minlat, minlon, maxlat, maxlon;
	float currTimestamp,lastTimestamp;
	double updateDist; //here in rad
	int cyear,cmonth,cday,chour,cmin,csec; //creation time of the track file
	char *filename;
	long trackPointCounter;
	FILE *tracklogFile;
	enum blackBoxStatus status;
};

bool openRecordingFile(void);
void recordPos(double lat, double lon, float timestamp, int year, int month, int day, int hour, int min, float sec);

static struct BlackBoxStruct BlackBox = {
	.filename=NULL,
	.tracklogFile=NULL,
	.status=BBS_NOT_SET
};

void BlackBoxStart(void) {
	if(BlackBox.status!=BBS_NOT_SET) return;
	BlackBox.lastTimestamp=-config.recordTimeInterval;
	BlackBox.trackPointCounter=0;
	BlackBox.minlat=90;
	BlackBox.minlon=180;
	BlackBox.maxlat=-90;
	BlackBox.maxlon=-180;
	BlackBox.updateDist=m2Rad(config.recordMinDist);
	if(BlackBox.updateDist<MIN_DIST) BlackBox.updateDist=MIN_DIST;
	struct tm time_str;
	long currTime=time(NULL); //get current time from internal clock
	time_str=*localtime(&currTime); //to obtain day, month, year, hour and minute
	BlackBox.cyear=time_str.tm_year+1900; //year time of creation of the track file
	BlackBox.cmonth=time_str.tm_mon+1; //month
	BlackBox.cday=time_str.tm_mday; //creation day
	BlackBox.chour=time_str.tm_hour; //hour
	BlackBox.cmin=time_str.tm_min; //minute
	BlackBox.csec=time_str.tm_sec;
	asprintf(&BlackBox.filename,"Track_%d-%02d-%02d_%02d-%02d-%02d.gpx",BlackBox.cyear,BlackBox.cmonth,BlackBox.cday,BlackBox.chour,BlackBox.cmin,BlackBox.csec);
	BlackBox.status=BBS_WAIT_FIX;
}

bool openRecordingFile(void) {
	char *path;
	asprintf(&path,"%sTracks/%s",BASE_PATH,BlackBox.filename);
	BlackBox.tracklogFile=fopen(path,"w");
	free(path);
	if(BlackBox.tracklogFile==NULL) {
		printLog("BlackBox ERROR: Unable to write track file.\n");
		BlackBoxClose();
		return false;
	}
	fprintf(BlackBox.tracklogFile,"<?xml version=\"1.0\" encoding=\"ISO-8859-1\" standalone=\"yes\"?>\n"
			"<gpx version=\"1.1\" creator=\"AirNavigator by Alus.it\"\n"
			"xmlns=\"http://www.topografix.com/GPX/1/1\"\n"
			"xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n"
			"xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\n"
			"<trk>\n"
			"<name>Track created on %d/%02d/%4d at %2d:%02d:%02d</name>\n"
			"<src>%s - Device serial number ID: %s</src>\n"
			"<trkseg>\n",BlackBox.cday,BlackBox.cmonth,BlackBox.cyear,BlackBox.chour,BlackBox.cmin,BlackBox.csec,config.tomtomModel,config.serialNumber);
	BlackBox.status=BBS_WAIT_POS;
	return true;
}

void BlackBoxPause(void) {
	if(BlackBox.status==BBS_WAIT_OPT) BlackBoxCommit();
	BlackBox.status=BBS_PAUSED;
}

void BlackBoxResume(void) {
	if(BlackBox.status==BBS_PAUSED && BlackBox.tracklogFile!=NULL) BlackBox.status=BBS_WAIT_POS;
}

bool BlackBoxIsStarted(void) {
	return(BlackBox.status!=BBS_NOT_SET);
}

bool BlackBoxIsPaused(void) {
	return(BlackBox.status==BBS_PAUSED);
}

void recordPos(double lat, double lon, float timestamp, int year, int month, int day, int hour, int min, float sec) {
	BlackBox.trackPointCounter++;
	BlackBox.lastlat=lat;
	BlackBox.lastlon=lon;
	BlackBox.lastTimestamp=timestamp;
	lat=Rad2Deg(lat);
	lon=-Rad2Deg(lon); //For the GPX standard East longitudes are positive
	if(lat<BlackBox.minlat) BlackBox.minlat=lat;
	if(lon<BlackBox.minlon) BlackBox.minlon=lon;
	if(lat>BlackBox.maxlat) BlackBox.maxlat=lat;
	if(lon>BlackBox.maxlon) BlackBox.maxlon=lon;
	fprintf(BlackBox.tracklogFile,"<trkpt lat=\"%f\" lon=\"%f\">\n"
			"<time>%d-%02d-%02dT%02d:%02d:%06.3fZ</time>\n",lat,lon,year,month,day,hour,min,sec);
	BlackBox.status=BBS_WAIT_OPT;
}

bool BlackBoxRecordPos(double lat, double lon, float timestamp, int hour, int min, float sec, int day, int month, int year, bool dateChanged) {
	bool retval=false;
	switch(BlackBox.status) {
		case BBS_WAIT_FIX:
			if(openRecordingFile()) recordPos(lat,lon,timestamp,year,month,day,hour,min,sec);
			break;
		case BBS_WAIT_POS: {
			double deltaT;
			if(dateChanged) BlackBox.lastTimestamp-=86400;
			if(timestamp>BlackBox.lastTimestamp) deltaT=timestamp-BlackBox.lastTimestamp;
			else {
				retval=false;
				break;
			}
			if(deltaT>=config.recordTimeInterval) {
				double deltaS;
				deltaS=calcAngularDist(lat,lon,BlackBox.lastlat,BlackBox.lastlon);
				if(deltaS>=BlackBox.updateDist) {
					recordPos(lat,lon,timestamp,year,month,day,hour,min,sec);
					retval=true;
				}
			}
		} break;
		case BBS_WAIT_OPT:
			BlackBoxCommit();
			retval=BlackBoxRecordPos(lat,lon,timestamp,hour,min,sec,day,month,year,dateChanged);
			break;
		case BBS_NOT_SET: //do nothing
		case BBS_PAUSED: //do nothing
			break;
	}
	return retval;
}

bool BlackBoxRecordAlt(double alt) {
	if(BlackBox.status!=BBS_WAIT_OPT) return false;
	fprintf(BlackBox.tracklogFile,"<ele>%.1f</ele>\n",alt);
	return true;
}

bool BlackBoxRecordSpeed(double speed) { //Speed in m/s
	if(BlackBox.status!=BBS_WAIT_OPT) return false;
	fprintf(BlackBox.tracklogFile,"<speed>%.2f</speed>\n",speed);
	return true;
}

bool BlackBoxRecordCourse(double course) {
	if(BlackBox.status!=BBS_WAIT_OPT) return false;
	fprintf(BlackBox.tracklogFile,"<course>%.2f</course>\n",course);
	return true;
}

bool BlackBoxCommit(void) {
	if(BlackBox.status!=BBS_WAIT_OPT) return false;
	fprintf(BlackBox.tracklogFile,"</trkpt>\n");
	BlackBox.status=BBS_WAIT_POS;
	return true;
}

void BlackBoxClose(void) {
	BlackBoxCommit();
	BlackBox.status=BBS_NOT_SET;
	if(BlackBox.tracklogFile!=NULL) {
		fprintf(BlackBox.tracklogFile,"</trkseg>\n"
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
				"</gpx>",BlackBox.trackPointCounter,BlackBox.filename,BlackBox.cyear,BlackBox.cmonth,BlackBox.cday,BlackBox.chour,BlackBox.cmin,BlackBox.csec,BlackBox.minlat,BlackBox.minlon,BlackBox.maxlat,BlackBox.maxlon);
		fclose(BlackBox.tracklogFile);
	}
	free(BlackBox.filename);
	BlackBox.filename=NULL;
}

