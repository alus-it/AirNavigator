//============================================================================
// Name        : BlackBox.c
// Since       : 14/9/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 3/11/2013
// Description : Produces tracelogFiles as XML GPX files
//============================================================================

//FIXME: problem detected: it happened only one time, the first 5-10 trkpnt's had wrong timestamp but creation time of track file was correct

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "BlackBox.h"
#include "Configuration.h"
#include "AirCalc.h"

double lastlat, lastlon, lastalt, minlat, minlon, maxlat, maxlon;
float currTimestamp,lastTimestamp;
double updateDist; //here in rad
int cyear,cmonth,cday,chour,cmin,csec; //creation time of the track file
char *filename=NULL;
long trackPointCounter;
FILE *tracklogFile=NULL;
short bbStatus=BBS_NOT_SET;

//TODO: here we should put a mutex

void BlackBoxStart() {
	if(bbStatus!=BBS_NOT_SET) return;
	trackPointCounter=0;
	minlat=90;
	minlon=180;
	maxlat=-90;
	maxlon=-180;
	lastTimestamp=-config.recordTimeInterval;
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

short openRecordingFile() {
	if(bbStatus!=BBS_WAIT_FIX) return 0;
	char *path;
	asprintf(&path,"/mnt/sdcard/AirNavigator/Tracks/%s",filename);
	tracklogFile=fopen(path,"w");
	free(path);
	if(tracklogFile==NULL) {
		BlackBoxClose();
		return -1;
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
	return 1;
}

void BlackBoxPause() {
	if(bbStatus==BBS_WAIT_OPT) BlackBoxCommit();
	bbStatus=BBS_PAUSED;
}

void BlackBoxResume() {
	if(bbStatus==BBS_PAUSED && tracklogFile!=NULL) bbStatus=BBS_WAIT_POS;
}

short BlackBoxRecordPos(double lat, double lon, float timestamp, int hour, int min, float sec) {
	short retval=0;
	switch(bbStatus) {
		case BBS_WAIT_POS: {
			double deltaT;
			if(timestamp>lastTimestamp) deltaT=timestamp-lastTimestamp;
			else if(timestamp!=lastTimestamp) {
				deltaT=timestamp+86400-lastTimestamp; //update the date!
				long currTime=time(NULL)+10800; //get current time + 3h just to avoid errors
				struct tm time_str;
				time_str=*localtime(&currTime); //to obtain day, month, year
				cyear=time_str.tm_year+1900; //year
				cmonth=time_str.tm_mon+1; //month
				cday=time_str.tm_mday; //day
			} else break;
			if(deltaT>=config.recordTimeInterval) {
				double deltaS;
				deltaS=calcAngularDist(lat,lon,lastlat,lastlon);
				if(deltaS>=updateDist) {
					trackPointCounter++;
					lastlat=lat;
					lastlon=lon;
					lastTimestamp=timestamp;
					lat=Rad2Deg(lat);
					lon=-Rad2Deg(lon); //For GPX standard East longitudes are positive
					if(lat<minlat) minlat=lat;
					if(lon<minlon) minlon=lon;
					if(lat>maxlat) maxlat=lat;
					if(lon>maxlon) maxlon=lon;
					fprintf(tracklogFile,"<trkpt lat=\"%f\" lon=\"%f\">\n"
						"<time>%d-%02d-%02dT%02d:%02d:%06.3fZ</time>\n",lat,lon,cyear,cmonth,cday,hour,min,sec);
					bbStatus=BBS_WAIT_OPT;
					retval=1;
				}
			}
		}
			break;
		case BBS_WAIT_OPT:
			retval=BlackBoxCommit();
			break;
		case BBS_WAIT_FIX:
			retval=openRecordingFile(lat,lon,timestamp,hour,min,sec);
			if(retval==1) retval=BlackBoxRecordPos(lat,lon,timestamp,hour,min,sec); //if the opening of the track succeded the record the first point
			break;
		case BBS_NOT_SET: //do nothing
		case BBS_PAUSED: //do nothing
			break;
	}
	return retval;
}

short BlackBoxRecordAlt(double alt) {
	if(bbStatus!=BBS_WAIT_OPT) return 0;
	lastalt=alt;
	fprintf(tracklogFile,"<ele>%.1f</ele>\n",alt);
	return 1;
}

short BlackBoxRecordSpeed(double speed) { //Speed in m/s
	if(bbStatus!=BBS_WAIT_OPT) return 0;
	fprintf(tracklogFile,"<speed>%.2f</speed>\n",speed);
	return 1;
}

short BlackBoxRecordCourse(double course) {
	if(bbStatus!=BBS_WAIT_OPT) return 0;
	fprintf(tracklogFile,"<course>%.2f</course>\n",course);
	return 1;
}

short BlackBoxCommit() {
	if(bbStatus!=BBS_WAIT_OPT) return 0;
	fprintf(tracklogFile,"</trkpt>\n");
	bbStatus=BBS_WAIT_POS;
	return 1;
}

void BlackBoxClose() {
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

