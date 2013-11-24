//============================================================================
// Name        : NMEAparser.c
// Since       : 9/2/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 24/11/2013
// Description : Parses NMEA sentences from a GPS device
//============================================================================

//#define PARSE_ALL_SENTENCES
//#define PRINT_SENTENCES

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NMEAparser.h"
#include "Common.h"
#include "GPSreceiver.h"
#include "Navigator.h"
#include "AirCalc.h"
#include "BlackBox.h"
#include "FBrender.h"
#include "HSI.h"
#include "Geoidal.h"

struct NMEAparserDataStruct {
	float altTimestamp, dirTimestamp, newerTimestamp;
	bool GGAfound, RMCfound, GSAfound;
	int numOfGSVmsg, GSVmsgSeqNo, GSVsatSeqNo, GSVtotalSatInView;
	int rcvdBytesOfSentence;
	char sentence[MAX_SENTENCE_LENGTH];
	bool lineFeedExpected;
};

struct lastRedDataStruct {
	float alt, latMin, lonMin, timeSec, groundSpeedKnots, trueTrack, magneticVariation, pdop, hdop, vdop;
	char altUnit;
	int latGra, lonGra, timeHour, timeMin, SatsInView, SatsInUse, timeDay, timeMonth, timeYear;
	bool latNorth, lonEast, magneticVariationToEast;
};

int parseNMEAsentence(char* ascii);
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
unsigned int getCRCintValue(void);
short updateAltitude(float newAltitude, char altUnit, float timestamp);
void updateDirection(float newTrueTrack, float magneticVar, bool isVarToEast, float timestamp);
bool updatePosition(int newlatDeg, float newlatMin, bool newisLatN, int newlonDeg, float newlonMin, bool newisLonE, bool dateChaged);

struct NMEAparserDataStruct data = {
	.altTimestamp=0,
	.dirTimestamp=0,
	.newerTimestamp=0,
	.GGAfound=false,
	.RMCfound=false,
	.GSAfound=false,
	.numOfGSVmsg=0,
	.GSVmsgSeqNo=0,
	.GSVsatSeqNo=0,
	.GSVtotalSatInView=0,
	.rcvdBytesOfSentence=0,
	.lineFeedExpected=false
};

struct lastRedDataStruct lastRedData;

void NMEAparserProcessBuffer(unsigned char *buf, int redBytes) {
	for(int i=0;i<redBytes;i++) switch(buf[i]) {
		case '$':
			if(data.rcvdBytesOfSentence!=0) {
				data.rcvdBytesOfSentence=0;
				data.lineFeedExpected=false;
			}
			data.sentence[data.rcvdBytesOfSentence++]='$';
			break;
		case '\r':
			if(data.rcvdBytesOfSentence>0) data.lineFeedExpected=true;
			break;
		case '\n':
			if(data.lineFeedExpected && data.rcvdBytesOfSentence>3) { //to avoid overflow errors
				if(data.sentence[data.rcvdBytesOfSentence-3]=='*') { //CRC check
					unsigned int checksum=0;
					int j;
					for(j=1;j<data.rcvdBytesOfSentence-3;j++)
						checksum=checksum^data.sentence[j];
					if(getCRCintValue()==checksum) { //right CRC
#ifdef PRINT_SENTENCES //Print all the sentences received
						data.sentence[data.rcvdBytesOfSentence]='\0';
						printLog("%s\n",data.sentence);
#endif
						data.sentence[data.rcvdBytesOfSentence-3]='\0'; //put terminator to cut off checksum
						parseNMEAsentence(strdup(data.sentence));
					}
				} //end of CRC check, ignore all if CRC is wrong
				data.rcvdBytesOfSentence=0;
				data.lineFeedExpected=false;
			}
			break;
		default:
			if(data.rcvdBytesOfSentence>0) { //add chars to the sentence
				if(data.rcvdBytesOfSentence<MAX_SENTENCE_LENGTH) data.sentence[data.rcvdBytesOfSentence++]=buf[i];
				else { //to avoid buffer overflow
					data.rcvdBytesOfSentence=0;
					data.lineFeedExpected=false;
				}
			}
			break;
	} //end of switch(each byte) of just received sequence
	bool dateChanged=false, posChanged=false, altChanged=false;
	if(data.RMCfound) dateChanged=updateDate(lastRedData.timeDay,lastRedData.timeMonth,lastRedData.timeYear); //pre-check if date is changed
	if(data.GGAfound) {
		updateTime(data.newerTimestamp,lastRedData.timeHour,lastRedData.timeMin,lastRedData.timeSec,false); //updateTime must be done always before of updatePosition
		posChanged=updatePosition(lastRedData.latGra,lastRedData.latMin,lastRedData.latNorth,lastRedData.lonGra,lastRedData.lonMin,lastRedData.lonEast,dateChanged);
		altChanged=updateAltitude(lastRedData.alt,lastRedData.altUnit,data.newerTimestamp);
		updateNumOfTotalSatsInView(lastRedData.SatsInView);
		if(data.GSAfound) {
			updateNumOfActiveSats(lastRedData.SatsInUse);
			updateDiluition(lastRedData.pdop,lastRedData.hdop,lastRedData.vdop);
		} else updateHdiluition(lastRedData.hdop);
	}
	if(data.RMCfound) {
		if(!data.GGAfound) {
			updateTime(data.newerTimestamp,lastRedData.timeHour,lastRedData.timeMin,lastRedData.timeSec,false); //updateTime must be done always before of updatePosition
			posChanged=updatePosition(lastRedData.latGra,lastRedData.latMin,lastRedData.latNorth,lastRedData.lonGra,lastRedData.lonMin,lastRedData.lonEast,dateChanged);
		}
		updateSpeed(lastRedData.groundSpeedKnots);
		updateDirection(lastRedData.trueTrack,lastRedData.magneticVariation,lastRedData.magneticVariationToEast,data.newerTimestamp);
	}
	if(posChanged||altChanged) NavUpdatePosition(gps.lat,gps.lon,gps.realAltMt,gps.speedKmh,gps.trueTrack,gps.timestamp);
	FBrenderFlush();
	BlackBoxCommit();
	data.GGAfound=false;
	data.RMCfound=false;
	data.GSAfound=false;
}

int parseNMEAsentence(char* ascii) {
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
	if(r<0) printLog("WARNING: parsing sentence %s returned: %d\n",ascii,r);
	else if(r==2) printLog("Received unexpected sentence: %s\n",ascii);
#endif
	free(ascii);
	return 1;
}

bool updatePosition(int newlatDeg, float newlatMin, bool newisLatN, int newlonDeg, float newlonMin, bool newisLonE, bool dateChaged) {
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
		BlackBoxRecordPos(gps.lat,gps.lon,gps.timestamp,lastRedData.timeHour,lastRedData.timeMin,lastRedData.timeSec,dateChaged);
		return true;
	}
	return false;
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
		printLog("ERROR: Unknown altitude unit: %c\n",altUnit);
		return 0;
	}
	data.altTimestamp=timestamp;
	if(updateAlt) {
		gps.altMt=newAltitudeMt;
		gps.altFt=newAltitudeFt;
		double deltaMt=GeoidalGetSeparation(Rad2Deg(gps.lat),Rad2Deg(gps.lon));
		newAltitudeMt-=deltaMt;
		newAltitudeFt-=m2Ft(deltaMt);
		HSIdrawVSIscale(newAltitudeFt);
		PrintAltitude(newAltitudeMt,newAltitudeFt);
		if(data.altTimestamp!=0) {
			float deltaT;
			if(timestamp>data.altTimestamp) deltaT=timestamp-data.altTimestamp;
			else if(timestamp!=data.altTimestamp) {
				deltaT=timestamp+86400-data.altTimestamp;
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

void updateDirection(float newTrueTrack, float magneticVar, bool isVarToEast, float timestamp) {
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
			if(data.dirTimestamp!=0&&gps.speedKmh>10) {
				float deltaT;
				if(timestamp>data.dirTimestamp) deltaT=timestamp-data.dirTimestamp;
				else if(timestamp!=data.dirTimestamp) deltaT=timestamp+86400-data.dirTimestamp;
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
	data.dirTimestamp=timestamp;
}

int parseGGA(char* ascii) {
	int timeHour=-1;
	int timeMin=-1;
	float timeSec=0;
	int latGra=-1;
	float latMin=0;
	bool latNorth=true;
	int lonGra=-1;
	float lonMin=0;
	bool lonEast=true;
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
			latNorth=true;
			break;
		case 'S':
			latNorth=false;
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
			lonEast=true;
			break;
		case 'W':
			lonEast=false;
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
	if(timestamp<data.newerTimestamp) return 0; //the sentence is old
	if(quality!=Q_NO_FIX) {
		if(timestamp>data.newerTimestamp) { //this is a new one sentence
			data.newerTimestamp=timestamp;
			data.GGAfound=true;
			data.RMCfound=false;
			data.GSAfound=false;
		} else { //timestamp==newerTimestamp
			if(data.GGAfound) return 0;
			else data.GGAfound=true;
		}
		lastRedData.alt=alt;
		lastRedData.altUnit=altUnit;
		lastRedData.latGra=latGra;
		lastRedData.latMin=latMin;
		lastRedData.latNorth=latNorth;
		lastRedData.lonGra=lonGra;
		lastRedData.lonMin=lonMin;
		lastRedData.lonEast=lonEast;
		lastRedData.timeHour=timeHour;
		lastRedData.timeMin=timeMin;
		lastRedData.timeSec=timeSec;
		lastRedData.hdop=hDilutionPrecision;
		lastRedData.SatsInView=numOfSatellites;
		updateFixMode(MODE_GPS_FIX);
		return 1;
	} else { //there is no fix...
		updateFixMode(MODE_NO_FIX); //show that there is no fix
		if(timestamp>data.newerTimestamp) updateTime(timestamp,timeHour,timeMin,timeSec,true); //show the time
	}
	return 0;
}

int parseRMC(char* ascii) {
	int timeHour=-1,timeMin=-1,timeDay=-1,timeMonth=-1,timeYear=-1;
	float timeSec=0;
	bool isValid=false;
	int latGra=-1;
	float latMin=0;
	bool latNorth=true;
	int lonGra=-1;
	float lonMin=0;
	bool lonEast=true;
	float groundSpeedKnots=0;
	float trueTrack=0;
	float magneticVariation=0;
	bool magneticVariationToEast=true;
	char faa=FAA_ABSENT;
	char *field=strsep(&ascii,","); //Hour, Minute, second hhmmss.sss
	if(field==NULL) return (-1);
	if(strlen(field)>0) sscanf(field,"%2d%2d%f",&timeHour,&timeMin,&timeSec);
	field=strsep(&ascii,","); //Status
	if(field==NULL) return (-2);
	if(strlen(field)>0) switch(field[0]){
		case 'A':
			isValid=true;
			break;
		case 'V':
			isValid=false;
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
			latNorth=true;
			break;
		case 'S':
			latNorth=false;
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
			lonEast=true;
			break;
		case 'W':
			lonEast=false;
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
				magneticVariationToEast=true;
				break;
			case 'W':
				magneticVariationToEast=false;
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
			data.newerTimestamp=timestamp; //change of date
			data.GGAfound=false;
			data.GSAfound=false;
		}
		if(timestamp<data.newerTimestamp) return 0; //the sentence is old
		else {
			if(timestamp>data.newerTimestamp) { //this is a new one sentence
				data.newerTimestamp=timestamp;
				data.RMCfound=true;
				data.GGAfound=false;
				data.GSAfound=false;
			} else { //timestamp==newerTimestamp
				if(data.RMCfound) return 0;
				else data.RMCfound=true;
			}
			lastRedData.latGra=latGra;
			lastRedData.latMin=latMin;
			lastRedData.latNorth=latNorth;
			lastRedData.lonGra=lonGra;
			lastRedData.lonMin=lonMin;
			lastRedData.lonEast=lonEast;
			lastRedData.timeHour=timeHour;
			lastRedData.timeMin=timeMin;
			lastRedData.timeSec=timeSec;
			lastRedData.groundSpeedKnots=groundSpeedKnots;
			lastRedData.trueTrack=trueTrack;
			lastRedData.magneticVariation=magneticVariation;
			lastRedData.magneticVariationToEast=magneticVariationToEast;
			lastRedData.timeDay=timeDay;
			lastRedData.timeMonth=timeMonth;
			lastRedData.timeYear=timeYear;
			return 1;
		}
		return 0;
	}
	return 0;
}

int parseGSA(char* ascii) {
	bool autoSelectionMode;
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
			autoSelectionMode=true;
			break;
		case 'M':
			autoSelectionMode=false;
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
		if(data.GSAfound) return 0;
		else if(data.GGAfound && data.RMCfound) {
			data.GSAfound=1;
			lastRedData.pdop=pdop;
			lastRedData.hdop=hdop;
			lastRedData.vdop=vdop;
			lastRedData.SatsInUse=numOfSatellites;
			return 1;
		}
		return 0;
	}
	return 0;
}

int parseGSV(char* ascii) {
	char *field=strsep(&ascii,","); //Number of GSV messages
	if(field==NULL) {
		data.numOfGSVmsg=0;
		data.GSVmsgSeqNo=0;
		return (-1);
	}
	int num=0;
	if(strlen(field)>0) sscanf(field,"%d",&num);
	if(num==0) {
		data.numOfGSVmsg=0;
		data.GSVmsgSeqNo=0;
		return (-1);
	}
	field=strsep(&ascii,","); //GSV message seq no.
	if(field==NULL) {
		data.numOfGSVmsg=0;
		data.GSVmsgSeqNo=0;
		return (-2);
	}
	int seqNo=0;
	if(strlen(field)>0) sscanf(field,"%d",&seqNo);
	if(seqNo==0) {
		data.numOfGSVmsg=0;
		data.GSVmsgSeqNo=0;
		return (-2);
	}
	if(seqNo==1) { //the first one resets GSVmsgSeqNo counter
		data.numOfGSVmsg=0;
		data.GSVmsgSeqNo=0;
	}
	if(data.GSVmsgSeqNo==0) { //we are expecting the first
		data.numOfGSVmsg=num;
		data.GSVmsgSeqNo=1;
		field=strsep(&ascii,","); //total number of satellites in view
		if(field==NULL) return (-3);
		if(strlen(field)>0) sscanf(field,"%d",&data.GSVtotalSatInView);
		updateNumOfTotalSatsInView(data.GSVtotalSatInView);
	} else { //we are not expecting the first
		if(num!=data.numOfGSVmsg) {
			data.numOfGSVmsg=0;
			data.GSVmsgSeqNo=0;
			return (-1);
		}
		if(seqNo!=data.GSVmsgSeqNo+1) {
			data.numOfGSVmsg=0;
			data.GSVmsgSeqNo=0;
			return (-2);
		}
		data.GSVmsgSeqNo++;
		field=strsep(&ascii,","); //total number of satellites in view
		if(field==NULL) {
			data.numOfGSVmsg=0;
			data.GSVmsgSeqNo=0;
			return (-3);
		}
		if(strlen(field)>0) sscanf(field,"%d",&num);
		if(num!=data.GSVtotalSatInView) {
			data.numOfGSVmsg=0;
			data.GSVmsgSeqNo=0;
			return (-3);
		}
	}
	while(field!=NULL && data.GSVsatSeqNo<data.GSVtotalSatInView) {
		field=strsep(&ascii,","); //satellite PRN number
		if(field==NULL) break; //to exit from the sat sequence of current msg
		if(strlen(field)>0) sscanf(field,"%d",&gps.satellites[data.GSVsatSeqNo][SAT_PRN]);
		else gps.satellites[data.GSVsatSeqNo][SAT_PRN]=-1;
		field=strsep(&ascii,","); //elevation in degrees (00-90)
		if(field==NULL) {
			data.numOfGSVmsg=0;
			data.GSVmsgSeqNo=0;
			return (-5-data.GSVsatSeqNo*4);
		}
		if(strlen(field)>0) sscanf(field,"%d",&gps.satellites[data.GSVsatSeqNo][SAT_ELEVATION]);
		else gps.satellites[data.GSVsatSeqNo][SAT_ELEVATION]=-1;
		field=strsep(&ascii,","); //azimuth in degrees to 1 north (000-359)
		if(field==NULL) {
			data.numOfGSVmsg=0;
			data.GSVmsgSeqNo=0;
			return (-6-data.GSVsatSeqNo*4);
		}
		if(strlen(field)>0) sscanf(field,"%d",&gps.satellites[data.GSVsatSeqNo][SAT_AZIMUTH]);
		else gps.satellites[data.GSVsatSeqNo][SAT_AZIMUTH]=-1;
		field=strsep(&ascii,","); //SNR in dB (00-99)
		if(field!=NULL) {
			if(strlen(field)>0) sscanf(field,"%d",&gps.satellites[data.GSVsatSeqNo][SAT_SNR]);
		} else gps.satellites[data.GSVsatSeqNo][SAT_SNR]=-1;
		data.GSVsatSeqNo++;
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
		updateTime(timestamp,timeHour,timeMin,timeSec,false);
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
	bool latNorth=true;
	int lonGra=-1;
	float lonMin=0;
	bool lonEast=true;
	bool isValid=false;
	char faa=FAA_ABSENT;
	char *field=strsep(&ascii,","); //Latitude ddmm.mm
	if(field==NULL) return (-1);
	if(strlen(field)>0) sscanf(field,"%2d%f",&latGra,&latMin);
	field=strsep(&ascii,","); //North or South
	if(field==NULL) return (-2);
	if(strlen(field)>0) switch(field[0]){
		case 'N':
			latNorth=true;
			break;
		case 'S':
			latNorth=false;
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
			lonEast=true;
			break;
		case 'W':
			lonEast=false;
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
				isValid=true;
				break;
			case 'V':
				isValid=false;
				break;
			default:
				return (-6);
		}
		field=strsep(&ascii,","); //FAA Indicator (optional)
		if(field!=NULL) if(strlen(field)>0) faa=field[0];
	}
	if(isValid) {
		float timestamp=timeHour*3600+timeMin*60+timeSec;
		updatePosition(latGra,latMin,latNorth,lonGra,lonMin,lonEast,false);
		updateTime(timestamp,timeHour,timeMin,timeSec,false);
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

unsigned int getCRCintValue(void) {
	if(data.rcvdBytesOfSentence<9) return -1;
	char asciiCheksum[2]; //two digits
	asciiCheksum[0]=data.sentence[data.rcvdBytesOfSentence-2];
	asciiCheksum[1]=data.sentence[data.rcvdBytesOfSentence-1];
	unsigned int checksum;
	sscanf(asciiCheksum,"%x",&checksum); //read as hex
	return checksum;
}
