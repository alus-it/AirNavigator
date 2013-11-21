//============================================================================
// Name        : NMEAreader.c
// Since       : 9/2/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 21/11/2013
// Description : Reads from a NMEA serial device NMEA sentences and parse them
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>
#include <fcntl.h>
#include "AirNavigator.h"
#include "Configuration.h"
#include "AirCalc.h"
#include "Geoidal.h"
#include "NMEAreader.h"
#include "FBrender.h"
#include "HSI.h"
#include "Navigator.h"
#include "BlackBox.h"

#define BUFFER_SIZE         (4096*6)
#define MAX_SENTENCE_LENGTH 255

//#define PRINT_SENTENCES
//#define PARSE_ALL_SENTENCES

void updateNumOfTotalSatsInView(int totalSats);
void updateNumOfActiveSats(int workingSats);
void updateFixMode(int fixMode);
void configureNMEAreader(void);
unsigned int getCRCintValue(char* sentence, int rcvdBytesOfSentence);
short updateAltitude(float newAltitude, char altUnit, float timestamp);
short updateDate(int newDay, int newMonth, int newYear);
void updateTime(float timestamp, int newHour, int newMin, float newSec, short timeWithNoFix);
short updatePosition(int newlatDeg, float newlatMin, short newisLatN, int newlonDeg, float newlonMin, short newisLonE, short dateChaged);
void updateGroundSpeedAndDirection(float newSpeedKmh, float newSpeedKnots, float newTrueTrack, float newMagneticTrack);
void updateSpeed(float newSpeedKnots);
void updateDirection(float newTrueTrack, float magneticVar, short isVarToEast, float timestamp);
void updateHdiluition(float hDiluition);
void updateDiluition(float pDiluition, float hDiluition, float vDiluition);
void update(void);
int parseGGA(char* ascii);
int parseRMC(char* ascii);
int parseGSA(char* ascii);
int parseGSV(char* ascii);
#ifdef PARSE_ALL_SENTENCES
int parseGBS(char* ascii);
int parseMSS(char* ascii);
int parseZDA(char* ascii);
int parseVTG(char* ascii);
int parseGLL(char* ascii);
#endif
int parse(char* ascii, long timestamp);
void* run(void *ptr);

struct GPSdata gps = {
	.timestamp=-1,
	.speedKmh=-100,
	.speedKnots=-100,
	.altMt=-100,
	.altFt=-100,
	.realAltMt=-100,
	.realAltFt=-100,
	.trueTrack=-500,
	.day=-65,
	.second=-65,
	.latMinDecimal=-70,
	.lonMinDecimal=-70,
	.lat=100,
	.pdop=50,
	.hdop=50,
	.vdop=50,
	.fixMode=MODE_UNKNOWN
};

long BAUD;
int DATABITS,STOPBITS,PARITYON,PARITY;
pthread_t thread;
volatile short readingNMEA=-1; //-1 means still not initialized
float altTimestamp=0,dirTimestamp=0,newerTimestamp=0;
short GGAfound=0,RMCfound=0,GSAfound=0;
float altB,latMinB,lonMinB,timeSecB,groundSpeedKnotsB,trueTrackB,magneticVariationB,pdopB,hdopB,vdopB;
char altUnitB;
int latGraB,lonGraB,timeHourB,timeMinB,SatsInViewB,SatsInUseB,timeDayB,timeMonthB,timeYearB;
short latNorthB,lonEastB,magneticVariationToEastB;
int numOfGSVmsg=0,GSVmsgSeqNo=0,GSVsatSeqNo,GSVtotalSatInView;

void updateNumOfTotalSatsInView(int totalSats) {
	if(gps.satsInView!=totalSats) {
		gps.satsInView=totalSats;
		PrintNumOfSats(gps.activeSats,gps.satsInView);
	}
}

void updateNumOfActiveSats(int workingSats) {
	if(gps.activeSats!=workingSats) {
		gps.activeSats=workingSats;
		PrintNumOfSats(gps.activeSats,gps.satsInView);
	}
}

void updateFixMode(int fixMode) {
	if(gps.fixMode!=fixMode) {
		if(fixMode==MODE_GPS_FIX && (gps.fixMode==MODE_2D_FIX || gps.fixMode==MODE_3D_FIX)) return;
		gps.fixMode=fixMode;
		PrintFixMode(fixMode);
		if(fixMode==MODE_NO_FIX) FBrenderFlush(); //because we want to show it immediately
	}
}

void configureNMEAreader(void) {
	if(config.GPSdevName==NULL) config.GPSdevName=strdup("/var/run/gpsfeed"); //Default value
	switch(config.GPSbaudRate){ //configuration of the serial port
		case 230400:
			BAUD=B230400;
			break;
		case 115200:
		default:
			BAUD=B115200;
			break;
		case 57600:
			BAUD=B57600;
			break;
		case 38400:
			BAUD=B38400;
			break;
		case 19200:
			BAUD=B19200;
			break;
		case 9600:
			BAUD=B9600;
			break;
		case 4800:
			BAUD=B4800;
			break;
		case 2400:
			BAUD=B2400;
			break;
		case 1800:
			BAUD=B1800;
			break;
		case 1200:
			BAUD=B1200;
			break;
		case 600:
			BAUD=B600;
			break;
		case 300:
			BAUD=B300;
			break;
		case 200:
			BAUD=B200;
			break;
		case 150:
			BAUD=B150;
			break;
		case 134:
			BAUD=B134;
			break;
		case 110:
			BAUD=B110;
			break;
		case 75:
			BAUD=B75;
			break;
		case 50:
			BAUD=B50;
			break;
	} //end of switch baud_rate
	switch(config.GPSdataBits){
		case 8:
		default:
			DATABITS=CS8;
			break;
		case 7:
			DATABITS=CS7;
			break;
		case 6:
			DATABITS=CS6;
			break;
		case 5:
			DATABITS=CS5;
			break;
	} //end of switch data_bits
	switch(config.GPSstopBits){
		case 1:
		default:
			STOPBITS=0;
			break;
		case 2:
			STOPBITS=CSTOPB;
			break;
	} //end of switch stop bits
	switch(config.GPSparity){
		case 0:
		default: //none
			PARITYON=0;
			PARITY=0;
			break;
		case 1: //odd
			PARITYON=PARENB;
			PARITY=PARODD;
			break;
		case 2: //even
			PARITYON=PARENB;
			PARITY=0;
			break;
	} //end of switch parity
	GeoidalOpen();
	readingNMEA=0;
	updateNumOfTotalSatsInView(0); //Display: at the moment we have no info from GPS
	updateNumOfActiveSats(0);
	FBrenderFlush();
}

short NMEAreaderIsReading(void) {
	if(readingNMEA==-1) return 0; //in the case it is not initilized we are not reading..
	return readingNMEA;
}

unsigned int getCRCintValue(char* sentence, int rcvdBytesOfSentence) {
	if(rcvdBytesOfSentence<9) return -1;
	char asciiCheksum[2]; //two digits
	asciiCheksum[0]=sentence[rcvdBytesOfSentence-2];
	asciiCheksum[1]=sentence[rcvdBytesOfSentence-1];
	unsigned int checksum;
	sscanf(asciiCheksum,"%x",&checksum); //read as hex
	return checksum;
}

short updateAltitude(float newAltitude, char altUnit, float timestamp) {
	float newAltitudeMt=0,newAltitudeFt=0;
	short updateAlt=0;
	if(altUnit=='M' || altUnit=='m') {
		if(newAltitude!=gps.altMt) {
			updateAlt=1;
			newAltitudeMt=newAltitude;
			newAltitudeFt=m2Ft(newAltitude);
		}
	} else if(altUnit=='F' || altUnit=='f') {
		if(newAltitude!=gps.altFt) {
			updateAlt=1;
			newAltitudeFt=newAltitude;
			newAltitudeMt=Ft2m(newAltitude);
		}
	} else {
		logText("ERROR: Unknown altitude unit: %c\n",altUnit);
		return 0;
	}
	altTimestamp=timestamp;
	if(updateAlt) {
		gps.altMt=newAltitudeMt;
		gps.altFt=newAltitudeFt;
		double deltaMt=GeoidalGetSeparation(Rad2Deg(gps.lat),Rad2Deg(gps.lon));
		newAltitudeMt-=deltaMt;
		newAltitudeFt-=m2Ft(deltaMt);
		HSIdrawVSIscale(newAltitudeFt);
		PrintAltitude(newAltitudeMt,newAltitudeFt);
		if(altTimestamp!=0) {
			float deltaT;
			if(timestamp>altTimestamp) deltaT=timestamp-altTimestamp;
			else if(timestamp!=altTimestamp) {
				deltaT=timestamp+86400-altTimestamp;
				float deltaH=newAltitudeFt-gps.altFt;
				gps.climbFtMin=deltaH/(deltaT/60);
				PrintVerticalSpeed(gps.climbFtMin);
			}
		}
		gps.realAltMt=newAltitudeMt;
		gps.realAltFt=newAltitudeFt;
	} else if(gps.climbFtMin!=0) { //altitude remained the same: put the variometer to 0
		gps.climbFtMin=0;
		PrintVerticalSpeed(0);
	}
	BlackBoxRecordAlt(gps.realAltMt);
	return updateAlt;
}

short updateDate(int newDay, int newMonth, int newYear) {
	if(gps.day!=newDay) {
		gps.day=newDay;
		gps.month=newMonth;
		gps.year=newYear;
		if(gps.year<2000) gps.year+=2000;
		PrintDate(gps.day,gps.month,gps.year);
		return 1;
	}
	return 0;
}

void updateTime(float timestamp, int newHour, int newMin, float newSec, short timeWithNoFix) {
	if(gps.timestamp!=timestamp) {
		gps.timestamp=timestamp;
		gps.hour=newHour;
		gps.minute=newMin;
		gps.second=newSec;
		PrintTime(gps.hour,gps.minute,gps.second,timeWithNoFix);
		if(timeWithNoFix) FBrenderFlush();
	}
}

short updatePosition(int newlatDeg, float newlatMin, short newisLatN, int newlonDeg, float newlonMin, short newisLonE, short dateChaged) {
	if(gps.latMinDecimal!=newlatMin||gps.lonMinDecimal!=newlonMin) {
		gps.latDeg=newlatDeg;
		gps.lonDeg=newlonDeg;
		gps.latMinDecimal=newlatMin;
		gps.lonMinDecimal=newlonMin;
		gps.isLatN=newisLatN;
		gps.isLonE=newisLonE;
		gps.lat=latDegMin2rad(gps.latDeg,gps.latMinDecimal,gps.isLatN);
		gps.lon=lonDegMin2rad(gps.lonDeg,gps.lonMinDecimal,gps.isLonE);
		int latMin,lonMin;
		double latSec,lonSec;
		convertDecimal2DegMin(gps.latMinDecimal,&latMin,&latSec);
		convertDecimal2DegMin(gps.lonMinDecimal,&lonMin,&lonSec);
		PrintPosition(gps.latDeg,latMin,latSec,gps.isLatN,gps.lonDeg,lonMin,lonSec,gps.isLonE);
		BlackBoxRecordPos(gps.lat,gps.lon,gps.timestamp,timeHourB,timeMinB,timeSecB,dateChaged);
		return 1;
	}
	return 0;
}

void updateGroundSpeedAndDirection(float newSpeedKmh, float newSpeedKnots, float newTrueTrack, float newMagneticTrack) {
	if(newSpeedKnots!=gps.speedKnots) {
		gps.speedKnots=newSpeedKnots;
		gps.speedKmh=newSpeedKmh;
		PrintSpeed(newSpeedKmh,newSpeedKnots);
	}
	if(newSpeedKmh>2) if(newTrueTrack!=gps.trueTrack) {
		gps.trueTrack=newTrueTrack;
		gps.magneticTrack=newMagneticTrack;
		HSIupdateDir(newTrueTrack,newMagneticTrack);
	}
	BlackBoxRecordSpeed(Kmh2ms(newSpeedKmh));
	if(gps.speedKmh>4) BlackBoxRecordCourse(newTrueTrack);
}

void updateSpeed(float newSpeedKnots) {
	if(newSpeedKnots!=gps.speedKnots) {
		gps.speedKnots=newSpeedKnots;
		gps.speedKmh=Nm2Km(newSpeedKnots);
		PrintSpeed(gps.speedKmh,gps.speedKnots);
	}
	BlackBoxRecordSpeed(Kmh2ms(gps.speedKmh));
}

void updateDirection(float newTrueTrack, float magneticVar, short isVarToEast, float timestamp) {
	if(gps.speedKmh>2) {
		if(newTrueTrack!=gps.trueTrack) {
			gps.magneticVariation=magneticVar;
			gps.isMagVarToEast=isVarToEast;
			if(newTrueTrack<90&&newTrueTrack>270) { //sono sopra
				if(isVarToEast) gps.magneticTrack=newTrueTrack+magneticVar;
				else gps.magneticTrack=newTrueTrack-magneticVar;
			} else { //sono sotto
				if(isVarToEast) gps.magneticTrack=newTrueTrack-magneticVar;
				else gps.magneticTrack=newTrueTrack+magneticVar;
			}
			HSIupdateDir(newTrueTrack,gps.magneticTrack);
			if(dirTimestamp!=0&&gps.speedKmh>10) {
				float deltaT;
				if(timestamp>dirTimestamp) deltaT=timestamp-dirTimestamp;
				else if(timestamp!=dirTimestamp) deltaT=timestamp+86400-dirTimestamp;
				else return;
				float deltaA=newTrueTrack-gps.trueTrack;
				gps.turnRateDegSec=deltaA/deltaT;
				gps.turnRateDegMin=gps.turnRateDegSec*60;
				PrintTurnRate(gps.turnRateDegMin);
			}
			gps.trueTrack=newTrueTrack;
		} else {
			if(gps.turnRateDegSec!=0) {
				gps.turnRateDegSec=0;
				gps.turnRateDegMin=0;
				PrintTurnRate(0);
			}
		}
	}
	if(gps.speedKmh>4) BlackBoxRecordCourse(newTrueTrack);
	dirTimestamp=timestamp;
}

void updateHdiluition(float hDiluition) {
	if(gps.hdop!=hDiluition) {
		gps.hdop=hDiluition;
		PrintDiluitions(gps.pdop,gps.hdop,gps.vdop);
	}
}

void updateDiluition(float pDiluition, float hDiluition, float vDiluition) {
	if(gps.pdop!=pDiluition||gps.hdop!=hDiluition||gps.vdop!=vDiluition) {
		gps.pdop=pDiluition;
		gps.hdop=hDiluition;
		gps.vdop=vDiluition;
		PrintDiluitions(pDiluition,hDiluition,vDiluition);
	}
}

void update(void) {
	short dateChanged=0,posChanged=0,altChanged=0;
	if(RMCfound) dateChanged=updateDate(timeDayB,timeMonthB,timeYearB); //pre-check if date is changed
	if(GGAfound) {
		updateTime(newerTimestamp,timeHourB,timeMinB,timeSecB,0); //updateTime must be done always before of updatePosition
		posChanged=updatePosition(latGraB,latMinB,latNorthB,lonGraB,lonMinB,lonEastB,dateChanged);
		altChanged=updateAltitude(altB,altUnitB,newerTimestamp);
		updateNumOfTotalSatsInView(SatsInViewB);
		if(GSAfound) {
			updateNumOfActiveSats(SatsInUseB);
			updateDiluition(pdopB,hdopB,vdopB);
		} else updateHdiluition(hdopB);
	}
	if(RMCfound) {
		if(!GGAfound) {
			updateTime(newerTimestamp,timeHourB,timeMinB,timeSecB,0); //updateTime must be done always before of updatePosition
			posChanged=updatePosition(latGraB,latMinB,latNorthB,lonGraB,lonMinB,lonEastB,dateChanged);
		}
		updateSpeed(groundSpeedKnotsB);
		updateDirection(trueTrackB,magneticVariationB,magneticVariationToEastB,newerTimestamp);
	}
	if(posChanged||altChanged) NavUpdatePosition(gps.lat,gps.lon,gps.realAltMt,gps.speedKmh,gps.trueTrack,gps.timestamp);
	FBrenderFlush();
	BlackBoxCommit();
	GGAfound=0;
	RMCfound=0;
	GSAfound=0;
}

int parseGGA(char* ascii) {
	int timeHour=-1;
	int timeMin=-1;
	float timeSec=0;
	int latGra=-1;
	float latMin=0;
	short latNorth=1;
	int lonGra=-1;
	float lonMin=0;
	short lonEast=1;
	int quality=Q_NO_FIX;
	int numOfSatellites=-1;
	float hDilutionPrecision=0;
	float alt=0;
	char altUnit;
	float geoidalSeparation=0;
	char geoidalUnit;
	float diffAge=0;
	int diffRef=-1;
	char *field=strsep(&ascii,","); //Hour, Minute, second hhmmss.sss
	if(field==NULL) return (-1);
	if(strlen(field)>0) sscanf(field,"%2d%2d%f",&timeHour,&timeMin,&timeSec);
	field=strsep(&ascii,","); //Latitude ddmm.mm
	if(field==NULL) return (-2);
	if(strlen(field)>0) sscanf(field,"%2d%f",&latGra,&latMin);
	field=strsep(&ascii,","); //North or South
	if(field==NULL) return (-3);
	if(strlen(field)>0) switch(field[0]){
		case 'N':
			latNorth=1;
			break;
		case 'S':
			latNorth=0;
			break;
		default:
			return (-3);
	}
	field=strsep(&ascii,","); //Longitude dddmm.mm
	if(field==NULL) return (-4);
	if(strlen(field)>0) sscanf(field,"%3d%f",&lonGra,&lonMin);
	field=strsep(&ascii,","); //East or West
	if(field==NULL) return (-3);
	if(strlen(field)>0) switch(field[0]){
		case 'E':
			lonEast=1;
			break;
		case 'W':
			lonEast=0;
			break;
		default:
			return (-5);
	}
	field=strsep(&ascii,","); //Quality
	if(field==NULL) return (-6);
	if(strlen(field)>0) quality=field[0]-'0';
	field=strsep(&ascii,","); //Number of satellites
	if(field==NULL) return (-7);
	if(strlen(field)>0) sscanf(field,"%d",&numOfSatellites);
	field=strsep(&ascii,","); //H dilution
	if(field==NULL) return (-8);
	if(strlen(field)>0) sscanf(field,"%f",&hDilutionPrecision);
	field=strsep(&ascii,","); //Altitude
	if(field==NULL) return (-9);
	if(strlen(field)>0) sscanf(field,"%f",&alt);
	field=strsep(&ascii,","); //Altitude unit
	if(field==NULL) return (-10);
	altUnit=field[0];
	field=strsep(&ascii,","); //Geoidal separation
	if(field==NULL) return (-11);
	if(strlen(field)>0) sscanf(field,"%f",&geoidalSeparation);
	field=strsep(&ascii,","); //Geoidal Separation unit
	if(field==NULL) return (-12);
	geoidalUnit=field[0];
	field=strsep(&ascii,","); //Age of differential GPS data
	if(field==NULL) return (-13);
	if(strlen(field)>0) sscanf(field,"%f",&diffAge);
	field=strsep(&ascii,","); //Differential reference station ID, the last one
	if(field!=NULL) if(strlen(field)>0) sscanf(field,"%d",&diffRef);
	float timestamp=timeHour*3600+timeMin*60+timeSec;
	if(timestamp<newerTimestamp) return 0; //the sentence is old
	if(quality!=Q_NO_FIX) {
		if(timestamp>newerTimestamp) { //this is a new one sentence
			newerTimestamp=timestamp;
			GGAfound=1;
			RMCfound=0;
			GSAfound=0;
		} else { //timestamp==newerTimestamp
			if(GGAfound) return 0;
			else GGAfound=1;
		}
		altB=alt;
		altUnitB=altUnit;
		latGraB=latGra;
		latMinB=latMin;
		latNorthB=latNorth;
		lonGraB=lonGra;
		lonMinB=lonMin;
		lonEastB=lonEast;
		timeHourB=timeHour;
		timeMinB=timeMin;
		timeSecB=timeSec;
		hdopB=hDilutionPrecision;
		SatsInViewB=numOfSatellites;
		updateFixMode(MODE_GPS_FIX);
		return 1;
	} else { //there is no fix...
		updateFixMode(MODE_NO_FIX); //show that there is no fix
		if(timestamp>newerTimestamp) updateTime(timestamp,timeHour,timeMin,timeSec,1); //show the time
	}
	return 0;
}

int parseRMC(char* ascii) {
	int timeHour=-1,timeMin=-1,timeDay=-1,timeMonth=-1,timeYear=-1;
	float timeSec=0;
	short isValid=0;
	int latGra=-1;
	float latMin=0;
	short latNorth=1;
	int lonGra=-1;
	float lonMin=0;
	short lonEast=1;
	float groundSpeedKnots=0;
	float trueTrack=0;
	float magneticVariation=0;
	short magneticVariationToEast=1;
	char faa=FAA_ABSENT;
	char *field=strsep(&ascii,","); //Hour, Minute, second hhmmss.sss
	if(field==NULL) return (-1);
	if(strlen(field)>0) sscanf(field,"%2d%2d%f",&timeHour,&timeMin,&timeSec);
	field=strsep(&ascii,","); //Status
	if(field==NULL) return (-2);
	if(strlen(field)>0) switch(field[0]){
		case 'A':
			isValid=1;
			break;
		case 'V':
			isValid=0;
			break;
		default:
			return (-2);
	}
	field=strsep(&ascii,","); //Latitude ddmm.mm
	if(field==NULL) return (-3);
	if(strlen(field)>0) sscanf(field,"%2d%f",&latGra,&latMin);
	field=strsep(&ascii,","); //North or South
	if(field==NULL) return (-4);
	if(strlen(field)>0) switch(field[0]){
		case 'N':
			latNorth=1;
			break;
		case 'S':
			latNorth=0;
			break;
		default:
			return (-4);
	}
	field=strsep(&ascii,","); //Longitude dddmm.mm
	if(field==NULL) return (-5);
	if(strlen(field)>0) sscanf(field,"%3d%f",&lonGra,&lonMin);
	field=strsep(&ascii,","); //East or West
	if(field==NULL) return (-6);
	if(strlen(field)>0) switch(field[0]){
		case 'E':
			lonEast=1;
			break;
		case 'W':
			lonEast=0;
			break;
		default:
			return (-6);
	}
	field=strsep(&ascii,","); //Ground speed Knots
	if(field==NULL) return (-7);
	if(strlen(field)>0) sscanf(field,"%f",&groundSpeedKnots);
	field=strsep(&ascii,","); //True track
	if(field==NULL) return (-8);
	if(strlen(field)>0) sscanf(field,"%f",&trueTrack);
	field=strsep(&ascii,","); //Date ddmmyy
	if(field==NULL) return (-9);
	if(strlen(field)>=6) sscanf(field,"%2d%2d%d",&timeDay,&timeMonth,&timeYear);
	field=strsep(&ascii,","); //Magnetic declination
	if(field==NULL) return (-10);
	if(strlen(field)>0) sscanf(field,"%f",&magneticVariation);
	field=strsep(&ascii,","); //Magnetic declination East or West, can be the last one
	if(field!=NULL) {
		if(strlen(field)>0) switch(field[0]){
			case 'E':
				magneticVariationToEast=1;
				break;
			case 'W':
				magneticVariationToEast=0;
				break;
			default:
				return (-11);
		}
		field=strsep(&ascii,","); //FAA Indicator (optional)
		if(field!=NULL) if(strlen(field)>0) faa=field[0];
	}
	if(isValid) {
		float timestamp=timeHour*3600+timeMin*60+timeSec;
		if(gps.day!=-65&&timeDay!=gps.day) {
			newerTimestamp=timestamp; //change of date
			GGAfound=0;
			GSAfound=0;
		}
		if(timestamp<newerTimestamp) return 0; //the sentence is old
		else {
			if(timestamp>newerTimestamp) { //this is a new one sentence
				newerTimestamp=timestamp;
				RMCfound=1;
				GGAfound=0;
				GSAfound=0;
			} else { //timestamp==newerTimestamp
				if(RMCfound) return 0;
				else RMCfound=1;
			}
			latGraB=latGra;
			latMinB=latMin;
			latNorthB=latNorth;
			lonGraB=lonGra;
			lonMinB=lonMin;
			lonEastB=lonEast;
			timeHourB=timeHour;
			timeMinB=timeMin;
			timeSecB=timeSec;
			groundSpeedKnotsB=groundSpeedKnots;
			trueTrackB=trueTrack;
			magneticVariationB=magneticVariation;
			magneticVariationToEastB=magneticVariationToEast;
			timeDayB=timeDay;
			timeMonthB=timeMonth;
			timeYearB=timeYear;
			return 1;
		}
		return 0;
	}
	return 0;
}

int parseGSA(char* ascii) {
	short autoSelectionMode;
	int mode=MODE_NO_FIX;
	int satellites[12];
	int numOfSatellites=0;
	float pdop=0;
	float hdop=0;
	float vdop=0;
	char *field=strsep(&ascii,","); //Mode
	if(field==NULL) return (-1);
	if(strlen(field)>0) switch(field[0]){
		case 'A':
			autoSelectionMode=1;
			break;
		case 'M':
			autoSelectionMode=0;
			break;
		default:
			return (-1);
	}
	field=strsep(&ascii,","); //Signal Strength
	if(field==NULL) return (-2);
	if(strlen(field)>0) sscanf(field,"%d",&mode);
	short i;
	for(i=0;i<12;i++) { //Satellites IDs
		field=strsep(&ascii,",");
		if(field==NULL) return (-(i+3));
		if(strlen(field)>0) if(sscanf(field,"%d",&satellites[i])==1) numOfSatellites++;
	}
	field=strsep(&ascii,","); //PDOP
	if(field==NULL) return (-15);
	if(strlen(field)>0) sscanf(field,"%f",&pdop);
	field=strsep(&ascii,","); //HDOP
	if(field==NULL) return (-16);
	if(strlen(field)>0) sscanf(field,"%f",&hdop);
	field=strsep(&ascii,","); //VDOP, the last one
	if(field!=NULL) if(strlen(field)>0) sscanf(field,"%f",&vdop);
	updateFixMode(mode);
	if(mode!=MODE_NO_FIX) {
		if(GSAfound) return 0;
		else if(GGAfound&&RMCfound) {
			GSAfound=1;
			pdopB=pdop;
			hdopB=hdop;
			vdopB=vdop;
			SatsInUseB=numOfSatellites;
			return 1;
		}
		return 0;
	}
	return 0;
}

int parseGSV(char* ascii) {
	char *field=strsep(&ascii,","); //Number of GSV messages
	if(field==NULL) {
		numOfGSVmsg=0;
		GSVmsgSeqNo=0;
		return (-1);
	}
	int num=0;
	if(strlen(field)>0) sscanf(field,"%d",&num);
	if(num==0) {
		numOfGSVmsg=0;
		GSVmsgSeqNo=0;
		return (-1);
	}
	field=strsep(&ascii,","); //GSV message seq no.
	if(field==NULL) {
		numOfGSVmsg=0;
		GSVmsgSeqNo=0;
		return (-2);
	}
	int seqNo=0;
	if(strlen(field)>0) sscanf(field,"%d",&seqNo);
	if(seqNo==0) {
		numOfGSVmsg=0;
		GSVmsgSeqNo=0;
		return (-2);
	}
	if(seqNo==1) { //the first one resets GSVmsgSeqNo counter
		numOfGSVmsg=0;
		GSVmsgSeqNo=0;
	}
	if(GSVmsgSeqNo==0) { //we are expecting the first
		numOfGSVmsg=num;
		GSVmsgSeqNo=1;
		field=strsep(&ascii,","); //total number of satellites in view
		if(field==NULL) return (-3);
		if(strlen(field)>0) sscanf(field,"%d",&GSVtotalSatInView);
		updateNumOfTotalSatsInView(GSVtotalSatInView);
	} else { //we are not expecting the first
		if(num!=numOfGSVmsg) {
			numOfGSVmsg=0;
			GSVmsgSeqNo=0;
			return (-1);
		}
		if(seqNo!=GSVmsgSeqNo+1) {
			numOfGSVmsg=0;
			GSVmsgSeqNo=0;
			return (-2);
		}
		GSVmsgSeqNo++;
		field=strsep(&ascii,","); //total number of satellites in view
		if(field==NULL) {
			numOfGSVmsg=0;
			GSVmsgSeqNo=0;
			return (-3);
		}
		if(strlen(field)>0) sscanf(field,"%d",&num);
		if(num!=GSVtotalSatInView) {
			numOfGSVmsg=0;
			GSVmsgSeqNo=0;
			return (-3);
		}
	}
	while(field!=NULL&&GSVsatSeqNo<GSVtotalSatInView) {
		field=strsep(&ascii,","); //satellite PRN number
		if(field==NULL) break; //to exit from the sat sequence of current msg
		if(strlen(field)>0) sscanf(field,"%d",&gps.satellites[GSVsatSeqNo][SAT_PRN]);
		else gps.satellites[GSVsatSeqNo][SAT_PRN]=-1;
		field=strsep(&ascii,","); //elevation in degrees (00-90)
		if(field==NULL) {
			numOfGSVmsg=0;
			GSVmsgSeqNo=0;
			return (-5-GSVsatSeqNo*4);
		}
		if(strlen(field)>0) sscanf(field,"%d",&gps.satellites[GSVsatSeqNo][SAT_ELEVATION]);
		else gps.satellites[GSVsatSeqNo][SAT_ELEVATION]=-1;
		field=strsep(&ascii,","); //azimuth in degrees to 1 north (000-359)
		if(field==NULL) {
			numOfGSVmsg=0;
			GSVmsgSeqNo=0;
			return (-6-GSVsatSeqNo*4);
		}
		if(strlen(field)>0) sscanf(field,"%d",&gps.satellites[GSVsatSeqNo][SAT_AZIMUTH]);
		else gps.satellites[GSVsatSeqNo][SAT_AZIMUTH]=-1;
		field=strsep(&ascii,","); //SNR in dB (00-99)
		if(field!=NULL) {
			if(strlen(field)>0) sscanf(field,"%d",&gps.satellites[GSVsatSeqNo][SAT_SNR]);
		} else gps.satellites[GSVsatSeqNo][SAT_SNR]=-1;
		GSVsatSeqNo++;
	}
	return 1;
}

#ifdef PARSE_ALL_SENTENCES //A collection of others parse functions for others sentences that are not used/useful by the TomTom at the moment
int parseMSS(char* ascii) {
	int newSignalStrength=-1;
	int newSNR=-1;
	float newBeaconFreq=0;
	int newBeaconDataRate=-1;
	int newChannel=-1;
	char *field=strsep(&ascii,","); //Signal Strength
	if(field==NULL) return (-1);
	if(strlen(field)>0) sscanf(field,"%d",&newSignalStrength);
	field=strsep(&ascii,","); //SNR
	if(field==NULL) return (-2);
	if(strlen(field)>0) sscanf(field,"%d",&newSNR);
	field=strsep(&ascii,","); //Beacon Frequency
	if(field==NULL) return (-3);
	if(strlen(field)>0) sscanf(field,"%f",&newBeaconFreq);
	field=strsep(&ascii,","); //Beacon Data Rate
	if(field==NULL) return (-4);
	if(strlen(field)>0) sscanf(field,"%d",&newBeaconDataRate);
	field=strsep(&ascii,","); //Channel, the last one
	if(field!=NULL) if(strlen(field)>0) sscanf(field,"%d",&newChannel);
	if(newChannel!=-1) {
//		signalStrength=newSignalStrength;
//		SNR=newSNR;
//		beaconFrequency=newBeaconFreq;
//		beaconDataRate=newBeaconDataRate;
//		channel=newChannel;
		return 1;
	} else return 0;
}

int parseZDA(char* ascii) {
	int timeHour=-1,timeMin=-1,timeDay=-1,timeMonth=-1,timeYear=-1;
	float timeSec=0;
	int localZone=-1,localZoneMin=-1;
	char *field=strsep(&ascii,","); //Hour, Minute, second hhmmss.sss
	if(field==NULL) return (-1);
	if(strlen(field)>0) sscanf(field,"%2d%2d%f",&timeHour,&timeMin,&timeSec);
	field=strsep(&ascii,","); //Day
	if(field==NULL) return (-2);
	if(strlen(field)>0) sscanf(field,"%d",&timeDay);
	field=strsep(&ascii,","); //Month
	if(field==NULL) return (-3);
	if(strlen(field)>0) sscanf(field,"%d",&timeMonth);
	field=strsep(&ascii,","); //Year
	if(field==NULL) return (-4);
	if(strlen(field)>0) sscanf(field,"%d",&timeYear);
	field=strsep(&ascii,","); //Local zone description, 00 to +- 13 hours
	if(field==NULL) return (-2);
	if(strlen(field)>0) sscanf(field,"%d",&localZone);
	field=strsep(&ascii,","); // Local zone minutes, the last one
	if(field!=NULL) if(strlen(field)>0) sscanf(field,"%d",&localZoneMin);
	if(localZoneMin!=-1) {
		float timestamp=timeHour*3600+timeMin*60+timeSec;
		updateDate(timeDay,timeMonth,timeYear);
		updateTime(timestamp,timeHour,timeMin,timeSec,0);
		return 1;
	} else {
		return 0;
	}
}

int parseVTG(char* ascii) {
	float trueTrack=-1;
	float magneticTrack=-1;
	float groundSpeedKnots=-1;
	float groundSpeedKmh=-1;
	int faa=FAA_ABSENT;
	short old=1;
	char *field=strsep(&ascii,","); //True track
	if(field==NULL) return (-1);
	if(strlen(field)>0) sscanf(field,"%f",&trueTrack);
	field=strsep(&ascii,","); //Second field, we don't know now...
	if(field==NULL) return (-2);
	int len=strlen(field);
	char c;
	if(len>0) {
		if(len==1&&sscanf(field,"%c",&c)==1) if(c=='T') old=0; //new version
		if(old) sscanf(field,"%f",&magneticTrack); //old version: read Magnetic track
	}
	if(!old) { //New version
		field=strsep(&ascii,","); //Magnetic track
		if(field==NULL) return (-3);
		if(strlen(field)>0) sscanf(field,"%f",&magneticTrack);
		field=strsep(&ascii,","); //M
		if(field==NULL) return (-3);
		if(strlen(field)==1&&sscanf(field,"%c",&c)==1) {
			if(c!='M') return (-3);
		} else return (-3);
	}
	field=strsep(&ascii,","); //Ground Speed Knots, both versions
	if(field==NULL) return (-4);
	if(strlen(field)>0) sscanf(field,"%f",&groundSpeedKnots);
	if(!old) { //new version
		field=strsep(&ascii,","); //N
		if(field==NULL) return (-5);
		if(strlen(field)==1&&sscanf(field,"%c",&c)==1) {
			if(c!='N') return (-5);
		} else return (-5);
	}
	field=strsep(&ascii,","); //Ground Speed Km/h, both versions
	if(field!=NULL) {
		if(strlen(field)>0) sscanf(field,"%f",&groundSpeedKmh);
		if(!old) { //new version
			field=strsep(&ascii,","); //K
			if(field!=NULL) {
				if(strlen(field)==1&&sscanf(field,"%c",&c)==1) {
					if(c!='K') return (-7);
				} else return (-7);
			}
			field=strsep(&ascii,","); //FAA Indicator (optional)
			if(field!=NULL) if(strlen(field)>0) faa=field[0];
		}
	}
	if(faa!=FAA_NOTVAL&&groundSpeedKmh>=0) {
		updateGroundSpeedAndDirection(groundSpeedKmh,groundSpeedKnots,trueTrack,magneticTrack);
		return 1;
	} else return 0;
}

int parseGLL(char* ascii) {
	int timeHour=-1,timeMin=-1;
	float timeSec=0;
	int latGra=-1;
	float latMin=0;
	short latNorth=1;
	int lonGra=-1;
	float lonMin=0;
	short lonEast=1;
	short isValid=0;
	char faa=FAA_ABSENT;
	char *field=strsep(&ascii,","); //Latitude ddmm.mm
	if(field==NULL) return (-1);
	if(strlen(field)>0) sscanf(field,"%2d%f",&latGra,&latMin);
	field=strsep(&ascii,","); //North or South
	if(field==NULL) return (-2);
	if(strlen(field)>0) switch(field[0]){
		case 'N':
			latNorth=1;
			break;
		case 'S':
			latNorth=0;
			break;
		default:
			return (-2);
	}
	field=strsep(&ascii,","); //Longitude dddmm.mm
	if(field==NULL) return (-3);
	if(strlen(field)>0) sscanf(field,"%3d%f",&lonGra,&lonMin);
	field=strsep(&ascii,","); //East or West
	if(field==NULL) return (-4);
	if(strlen(field)>0) switch(field[0]){
		case 'E':
			lonEast=1;
			break;
		case 'W':
			lonEast=0;
			break;
		default:
			return (-4);
	}
	field=strsep(&ascii,","); //Hour, Minute, second hhmmss.sss
	if(field==NULL) return (-5);
	if(strlen(field)>0) sscanf(field,"%2d%2d%f",&timeHour,&timeMin,&timeSec);
	field=strsep(&ascii,","); //Status, can be the last one
	if(field!=NULL) {
		if(strlen(field)>0) switch(field[0]){
			case 'A':
				isValid=1;
				break;
			case 'V':
				isValid=0;
				break;
			default:
				return (-6);
		}
		field=strsep(&ascii,","); //FAA Indicator (optional)
		if(field!=NULL) if(strlen(field)>0) faa=field[0];
	}
	if(isValid) {
		float timestamp=timeHour*3600+timeMin*60+timeSec;
		updatePosition(latGra,latMin,latNorth,lonGra,lonMin,lonEast,0);
		updateTime(timestamp,timeHour,timeMin,timeSec,0);
		return 1;
	} else {
		return 0;
	}
}

int parseGBS(char* ascii) {
	int timeHour=-1,timeMin=-1;
	float timeSec=0;
	float latitudeError=0;
	float longitudeError=0;
	float altitudeError=0;
	int mostFailedSatPRN=-1;
	float missedDetectionProb=0;
	float estBiasOfFailedSat=0;
	float stdDevOfBias=0;
	char *field=strsep(&ascii,","); //Hour, Minute, second hhmmss.sss
	if(field==NULL) return (-1);
	if(strlen(field)>0) sscanf(field,"%2d%2d%f",&timeHour,&timeMin,&timeSec);
	field=strsep(&ascii,","); //Expected error in latitude (meters)
	if(field==NULL) return (-2);
	if(strlen(field)>0) sscanf(field,"%f",&latitudeError);
	field=strsep(&ascii,","); //Expected error in longitude (meters)
	if(field==NULL) return (-3);
	if(strlen(field)>0) sscanf(field,"%f",&longitudeError);
	field=strsep(&ascii,","); //Expected error in altitude (meters)
	if(field==NULL) return (-4);
	if(strlen(field)>0) sscanf(field,"%f",&altitudeError);
	field=strsep(&ascii,","); //PRN of most likely failed satellite
	if(field==NULL) return (-5);
	if(strlen(field)>0) sscanf(field,"%d",&mostFailedSatPRN);
	field=strsep(&ascii,","); //Probability of missed detection for most likely failed satellite
	if(field!=NULL) {
		if(strlen(field)>0) sscanf(field,"%f",&missedDetectionProb);
		field=strsep(&ascii,","); //Estimate of bias in meters on most likely failed satellite
		if(field!=NULL) {
			if(strlen(field)>0) sscanf(field,"%f",&estBiasOfFailedSat);
			field=strsep(&ascii,","); //Standard deviation of bias estimate
			if(field!=NULL) if(strlen(field)>0) sscanf(field,"%f",&stdDevOfBias);
		}
		return 1;
	}
	return 0;
}
#endif

int parse(char* ascii, long timestamp) {
	if(strlen(ascii)<6) return -1; //Received too short sentence
	int r=2;
	switch(ascii[3]){
		case 'G': //GGA GSA GSV GLL GBS
			switch(ascii[4]){
				case 'G': //GGA
					if(ascii[5]=='A') r=parseGGA(ascii+7);
					break;
				case 'S': //GSA GSV
					if(ascii[5]=='A') r=parseGSA(ascii+7); //GSA
					else if(ascii[5]=='V') r=parseGSV(ascii+7); //GSV
					break;
#ifdef PARSE_ALL_SENTENCES
				case 'L': //GLL
					if(ascii[5]=='L') r=parseGLL(ascii+7);
					break;
				case 'B': //GBS
					if(ascii[5]=='S') r=parseGBS(ascii+7);
					break;
#endif
			}
			break;
		case 'R': //RMC
			if(ascii[4]=='M'&&ascii[5]=='C') r=parseRMC(ascii+7);
			break;
#ifdef PARSE_ALL_SENTENCES
		case 'M': //MSS
			if(ascii[4]=='S'&&ascii[5]=='S') r=parseMSS(ascii+7);
			break;
		case 'Z': //ZDA
			if(ascii[4]=='D'&&ascii[5]=='A') r=parseZDA(ascii+7);
			break;
		case 'V': //VTG
			if(ascii[4]=='T'&&ascii[5]=='G') r=parseVTG(ascii+7);
			break;
#endif
	}
#ifdef PARSE_ALL_SENTENCES
	if(r<0) logText("WARNING: parsing sentence %s returned: %d\n",ascii,r);
	else if(r==2) logText("Received unexpected sentence: %s\n",ascii);
#endif
	free(ascii);
	return 1;
}

void* run(void *ptr) { //listening function, it will be ran in a separate thread
	static int fd=-1;
	fd=open(config.GPSdevName,O_RDONLY|O_NOCTTY|O_NONBLOCK); //read only, non blocking
	if(fd<0) {
		fd=-1;
		logText("ERROR: Can't open the gps serial port on device: %s\n",config.GPSdevName);
		readingNMEA=0;
		pthread_exit(NULL);
		return NULL;
	}
	struct termios oldtio,newtio; //place for old and new port settings for serial port
	tcgetattr(fd,&oldtio); // save current port settings
	newtio.c_cflag=BAUD|CRTSCTS|DATABITS|STOPBITS|PARITYON|PARITY|CLOCAL|CREAD; // set new port settings for canonical input processing
	newtio.c_iflag=IGNPAR;
	newtio.c_oflag=0;
	newtio.c_lflag=0; //ICANON;
	newtio.c_cc[VMIN]=1;
	newtio.c_cc[VTIME]=0;
	tcsetattr(fd,TCSANOW,&newtio); // Set the new options for the port...
	tcflush(fd,TCIFLUSH);
	char buf[BUFFER_SIZE]; //buffer for where data is put
	int maxfd=fd+1;;
	fd_set readfs;
	long i,redBytes;
	int rcvdBytesOfSentence=0;
	char sentence[MAX_SENTENCE_LENGTH]={0};
	long timestamp;
	short lineFeedExpected=1;
	while(readingNMEA) { // loop while waiting for input
		FD_SET(fd,&readfs);
		select(maxfd,&readfs,NULL,NULL,NULL); //wait to read because the read is now non-blocking
		if(readingNMEA) { //further check if we still want to read after waiting
			redBytes=read(fd,buf,BUFFER_SIZE);
			timestamp=time(NULL); //get the timestamp of last char sequence received
			for(i=0;i<redBytes;i++)
				switch(buf[i]){
					case '$':
						if(rcvdBytesOfSentence==0) sentence[rcvdBytesOfSentence++]='$';
						break;
					case '\r':
						if(rcvdBytesOfSentence>0) lineFeedExpected=1;
						break;
					case '\n':
						if(lineFeedExpected&&rcvdBytesOfSentence>3) { //to avoid overflow errors
							if(sentence[rcvdBytesOfSentence-3]=='*') { //CRC check
								unsigned int checksum=0;
								int j;
								for(j=1;j<rcvdBytesOfSentence-3;j++)
									checksum=checksum^sentence[j];
								if(getCRCintValue(sentence,rcvdBytesOfSentence)==checksum) { //right CRC
#ifdef PRINT_SENTENCES //Print all the sentences received
										sentence[rcvdBytesOfSentence]='\0';
										logText("%s\n",sentence);
#endif
									sentence[rcvdBytesOfSentence-3]='\0'; //put terminator cut off checksum
									parse(strdup(sentence),timestamp);
								}
							} //end of CRC check, ignore all if CRC is wrong
							rcvdBytesOfSentence=0;
							lineFeedExpected=0;
						}
						break;
					default:
						if(rcvdBytesOfSentence>0) { //add chars to the sentence
							if(rcvdBytesOfSentence<MAX_SENTENCE_LENGTH) sentence[rcvdBytesOfSentence++]=buf[i];
							else { //to avoid buffer overflow
								rcvdBytesOfSentence=0;
								lineFeedExpected=0;
							}
						}
						break;
				} //end of switch(each byte) of just received sequence
			update();
		} //end of further if(readingNMEA) check
	} //while readingNMEA
	tcsetattr(fd,TCSANOW,&oldtio); //restore old port settings
	if(fd>=0) close(fd); //close the serial port
	pthread_exit(NULL);
	return NULL;
}

short NMEAreaderStartRead(void) { //function to start the listening thread
	if(readingNMEA==-1) configureNMEAreader();
	if(!readingNMEA) {
		readingNMEA=1;
		if(pthread_create(&thread,NULL,run,(void*)NULL)) {
			readingNMEA=0;
			logText("NMEAreader: ERROR unable to create the reading thread.\n");
		}
	}
	return readingNMEA;
}

void NMEAreaderStopRead(void) {
	if(readingNMEA==1) readingNMEA=0;
}

void NMEAreaderClose(void) {
	NMEAreaderStopRead();
	GeoidalClose();
	pthread_join(thread,NULL); //wait for thread death
}
