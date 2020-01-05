//============================================================================
// Name        : NMEAparser.c
// Since       : 9/2/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : https://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2016 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/alus-it/AirNavigator.git
// Last change : 21/3/2016
// Description : Parses NMEA sentences from a GPS device
//============================================================================

#define PRINT_SENTENCES

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


#define MAX_FIELDS 30
#define MAX_FIELD_LENGTH 25


struct NMEAparserStruct {
	float altTimestamp, dirTimestamp, newerTimestamp, rcvdTimestamp;
	bool GGAfound, RMCfound, GSAfound;
	int numOfGSVmsg, GSVmsgSeqNo, GSVtotalSatInView;
	int rcvdBytesOfSentence, rcvdBytesOfField, rcvdBytesOfCheksum;
	char sentence[MAX_SENTENCE_LENGTH];
	char fields[MAX_FIELDS][MAX_FIELD_LENGTH];
	int fieldId;
	float alt, latMin, lonMin, timeSec, groundSpeedKnots, trueTrack, magneticVariation, pdop, hdop, vdop;
	char altUnit;
	int latGra, lonGra, timeHour, timeMin, SatsInView, SatsInUse, timeDay, timeMonth, timeYear;
	bool latNorth, lonEast, magneticVariationToEast;
};

int parseNMEAsentence(void);
int parseGGA(void);
int parseRMC(void);
int parseGSA(void);
int parseGSV(void);

unsigned int getCRCintValue(void);
bool updateAltitude(float newAltitude, char altUnit, float timestamp);
void updateDirection(float newTrueTrack, float magneticVar, bool isVarToEast, float timestamp);
bool updatePosition(int newlatDeg, float newlatMin, bool newisLatN, int newlonDeg, float newlonMin, bool newisLonE, bool dateChaged);
bool parseTime(char* field, int* timeHour, int* timeMin, float* timeSec);
bool parseDate(char* field, int* dd, int* mm, int* yy);
bool parseLatitude(char* firstField, char* secondField, int* latDeg, float* latMin, bool* latNorth);
bool parseLongitude(char* firstField, char* secondField, int* lonDeg, float* lonMin, bool* lonEast);
bool parseValid(char* field, bool* isValid);
bool parseEastWest(char* field, bool* isEast);
bool parseInteger(char* field, int* value);
bool parseFloat(char* field, float* value);

static struct NMEAparserStruct NMEAparser = {
	.altTimestamp=0,
	.dirTimestamp=0,
	.newerTimestamp=0,
	.rcvdTimestamp=0,
	.GGAfound=false,
	.RMCfound=false,
	.GSAfound=false,
	.numOfGSVmsg=0,
	.GSVmsgSeqNo=0,
	.GSVtotalSatInView=0,
	.rcvdBytesOfSentence=0,
	.rcvdBytesOfCheksum=-1,
	.fieldId=0
};

void NMEAparserProcessBuffer(unsigned char *buf, int redBytes) {
	for(int i=0;i<redBytes;i++) switch(buf[i]) {
		case '$':
			NMEAparser.rcvdBytesOfSentence=1;
			NMEAparser.rcvdBytesOfField=0;
			NMEAparser.rcvdBytesOfCheksum=-1;
			NMEAparser.fieldId=0;
			NMEAparser.sentence[0]='$';
			break;
		case ',':
			NMEAparser.fields[NMEAparser.fieldId][NMEAparser.rcvdBytesOfField]='\0';
			if(NMEAparser.rcvdBytesOfSentence > 5) {
				if(NMEAparser.fieldId<MAX_FIELDS) {
					if(NMEAparser.rcvdBytesOfSentence<MAX_SENTENCE_LENGTH) {
						NMEAparser.fieldId++;
						NMEAparser.rcvdBytesOfField=0;
						NMEAparser.sentence[NMEAparser.rcvdBytesOfSentence++]=',';
					}
				}
			}
			break;
		case '*':
			NMEAparser.fields[NMEAparser.fieldId][NMEAparser.rcvdBytesOfField]='\0';
			NMEAparser.rcvdBytesOfCheksum=0;
			if(NMEAparser.rcvdBytesOfSentence<MAX_SENTENCE_LENGTH) {
				NMEAparser.sentence[NMEAparser.rcvdBytesOfSentence++]='*';
			}
			break;
		case '\r':
		case '\n':
			break;
		default:
			if(NMEAparser.rcvdBytesOfSentence>0) {
				if(NMEAparser.rcvdBytesOfSentence<MAX_SENTENCE_LENGTH) {
					NMEAparser.sentence[NMEAparser.rcvdBytesOfSentence++]=buf[i]; //add chars to the sentence
					if(NMEAparser.rcvdBytesOfCheksum!=-1) { // checksum
						NMEAparser.rcvdBytesOfCheksum++;
						if(NMEAparser.rcvdBytesOfCheksum==2) { // do checksum check
							unsigned int checksum=0;
							for(int j=1;j<NMEAparser.rcvdBytesOfSentence-3;j++) checksum^=NMEAparser.sentence[j];
							if(getCRCintValue()==checksum) { //right CRC
								NMEAparser.sentence[NMEAparser.rcvdBytesOfSentence]='\0';
								#ifdef PRINT_SENTENCES //Print all the sentences received, if required
								printLog("%s\n",NMEAparser.sentence);
								#endif
								NMEAparser.rcvdTimestamp=getCurrentTime();
								parseNMEAsentence();
								NMEAparser.rcvdBytesOfSentence=0;
							}
						}
					} else { // not checksum
						NMEAparser.fields[NMEAparser.fieldId][NMEAparser.rcvdBytesOfField++]=buf[i];
					}
				} else { // overflow
					NMEAparser.rcvdBytesOfSentence=0; //to avoid buffer overflow
				}
			}
			break;
	} //end of switch(each byte) of just received sequence
	bool dateChanged=false, posChanged=false, altChanged=false;
	if(NMEAparser.GGAfound  || NMEAparser.RMCfound || NMEAparser.GSAfound) {
		pthread_mutex_lock(&gps.mutex);
		if(NMEAparser.RMCfound) dateChanged=updateDate(NMEAparser.timeDay,NMEAparser.timeMonth,NMEAparser.timeYear); //pre-check if date is changed
		if(NMEAparser.GGAfound) {
			updateTime(NMEAparser.newerTimestamp,NMEAparser.timeHour,NMEAparser.timeMin,NMEAparser.timeSec,false); //updateTime must be done always before of updatePosition
			posChanged=updatePosition(NMEAparser.latGra,NMEAparser.latMin,NMEAparser.latNorth,NMEAparser.lonGra,NMEAparser.lonMin,NMEAparser.lonEast,dateChanged);
			altChanged=updateAltitude(NMEAparser.alt,NMEAparser.altUnit,NMEAparser.newerTimestamp);
			updateNumOfTotalSatsInView(NMEAparser.SatsInView);
			if(NMEAparser.GSAfound) {
				updateNumOfActiveSats(NMEAparser.SatsInUse);
				//updateDiluition(NMEAparser.pdop,NMEAparser.hdop,NMEAparser.vdop);
			} //else updateHdiluition(NMEAparser.hdop);
		}
		if(NMEAparser.RMCfound) {
			if(!NMEAparser.GGAfound) {
				updateTime(NMEAparser.newerTimestamp,NMEAparser.timeHour,NMEAparser.timeMin,NMEAparser.timeSec,false); //updateTime must be done always before of updatePosition
				posChanged=updatePosition(NMEAparser.latGra,NMEAparser.latMin,NMEAparser.latNorth,NMEAparser.lonGra,NMEAparser.lonMin,NMEAparser.lonEast,dateChanged);
			}
			updateSpeed(NMEAparser.groundSpeedKnots);
			updateDirection(NMEAparser.trueTrack,NMEAparser.magneticVariation,NMEAparser.magneticVariationToEast,NMEAparser.newerTimestamp);
		}
		if(posChanged||altChanged) NavUpdatePosition(gps.lat,gps.lon,gps.realAltMt,gps.speedKmh,gps.timestamp);
		pthread_mutex_unlock(&gps.mutex);
		if(getMainStatus()==MAIN_DISPLAY_HSI) FBrenderFlush();
		BlackBoxCommit();
		NMEAparser.GGAfound=false;
		NMEAparser.RMCfound=false;
		NMEAparser.GSAfound=false;
	}
}

int parseNMEAsentence() {
	if(strlen(NMEAparser.sentence)<6) return -1; //wrong sentence
	int r=2;
	switch(NMEAparser.sentence[3]){
		case 'G': //GGA GSA GSV GLL GBS
			switch(NMEAparser.sentence[4]){
				case 'G': //GGA
					if(NMEAparser.sentence[5]=='A') {
						r=parseGGA();
						#ifdef PRINT_SENTENCES
						printLog("Time difference: %f\n",NMEAparser.newerTimestamp-NMEAparser.rcvdTimestamp);
						#endif
					}
					break;
				case 'S': //GSA GSV
					if(NMEAparser.sentence[5]=='A') r=parseGSA(); //GSA
					else if(NMEAparser.sentence[5]=='V') r=parseGSV(); //GSV
					break;
			}
			break;
		case 'R': //RMC
			if(NMEAparser.sentence[4]=='M' && NMEAparser.sentence[5]=='C') r=parseRMC();
			break;
	}
	#ifdef PRINT_SENTENCES
	if(r<0) printLog("WARNING: parsing sentence %s returned: %d\n",NMEAparser.sentence,r);
	else if(r==2) printLog("Received unexpected sentence: %s\n",NMEAparser.sentence);
	#endif
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
		if(getMainStatus()==MAIN_DISPLAY_HSI) {
			int latMin,lonMin;
			double latSec,lonSec;
			convertDecimal2DegMin(gps.latMinDecimal,&latMin,&latSec);
			convertDecimal2DegMin(gps.lonMinDecimal,&lonMin,&lonSec);
			PrintPosition(gps.latDeg,latMin,latSec,gps.isLatN,gps.lonDeg,lonMin,lonSec,gps.isLonE);
		} else if(getMainStatus()==MAIN_DISPLAY_SUNRISE_SUNSET) {
			//TODO: ....
		}
		BlackBoxRecordPos(gps.lat,gps.lon,gps.timestamp,gps.hour,gps.minute,gps.second,gps.day,gps.month,gps.year,dateChaged);
		return true;
	}
	return false;
}

bool updateAltitude(float newAltitude, char altUnit, float timestamp) {
	float newAltitudeMt=0,newAltitudeFt=0;
	bool updateAlt=false;
	if(altUnit=='M' || altUnit=='m') {
		if(newAltitude!=gps.altMt) {
			updateAlt=true;
			newAltitudeMt=newAltitude;
			newAltitudeFt=m2Ft(newAltitude);
		}
	} else if(altUnit=='F' || altUnit=='f') {
		if(newAltitude!=gps.altFt) {
			updateAlt=true;
			newAltitudeFt=newAltitude;
			newAltitudeMt=Ft2m(newAltitude);
		}
	} else {
		#ifdef PRINT_SENTENCES
		printLog("ERROR: Unknown altitude unit: %c\n",altUnit);
		#endif
		return 0;
	}
	NMEAparser.altTimestamp=timestamp;
	if(updateAlt) {
		gps.altMt=newAltitudeMt;
		gps.altFt=newAltitudeFt;
		double deltaMt=GeoidalGetSeparation(Rad2Deg(gps.lat),Rad2Deg(gps.lon));
		newAltitudeMt-=deltaMt;
		newAltitudeFt-=m2Ft(deltaMt);
		if(getMainStatus()==MAIN_DISPLAY_HSI) {
			HSIdrawVSIscale(newAltitudeFt);
			PrintAltitude(newAltitudeMt,newAltitudeFt);
		}
//		if(NMEAparser.altTimestamp!=0) {
//			float deltaT;
//			if(timestamp>NMEAparser.altTimestamp) deltaT=timestamp-NMEAparser.altTimestamp;
//			else if(timestamp!=NMEAparser.altTimestamp) {
//				deltaT=timestamp+86400-NMEAparser.altTimestamp;
//				float deltaH=newAltitudeFt-gps.altFt;
//				gps.climbFtMin=deltaH/(deltaT/60);
//				if(getMainStatus()==MAIN_DISPLAY_HSI) PrintVerticalSpeed(gps.climbFtMin);
//			}
//		}
		gps.realAltMt=newAltitudeMt;
		gps.realAltFt=newAltitudeFt;
	}
//else if(gps.climbFtMin!=0) { //altitude remained the same: put the variometer to 0
//		gps.climbFtMin=0;
//		if(getMainStatus()==MAIN_DISPLAY_HSI) PrintVerticalSpeed(0);
//	}
	BlackBoxRecordAlt(gps.realAltMt);
	return updateAlt;
}

void updateDirection(float newTrueTrack, float magneticVar, bool isVarToEast, float timestamp) {
	if(gps.speedKmh>2) {
		if(newTrueTrack!=gps.trueTrack) {
			gps.magneticVariation=magneticVar;
			gps.isMagVarToEast=isVarToEast;
			if(newTrueTrack<90 && newTrueTrack>270) { // I'm up
				if(isVarToEast) gps.magneticTrack=newTrueTrack+magneticVar;
				else gps.magneticTrack=newTrueTrack-magneticVar;
			} else { // I'm down
				if(isVarToEast) gps.magneticTrack=newTrueTrack-magneticVar;
				else gps.magneticTrack=newTrueTrack+magneticVar;
			}
			if(getMainStatus()==MAIN_DISPLAY_HSI) HSIupdateDir(newTrueTrack);
			if(NMEAparser.dirTimestamp!=0 && gps.speedKmh>10) {
				float deltaT;
				if(timestamp>NMEAparser.dirTimestamp) deltaT=timestamp-NMEAparser.dirTimestamp;
				else if(timestamp!=NMEAparser.dirTimestamp) deltaT=timestamp+86400-NMEAparser.dirTimestamp;
				else return;
//				float deltaA=newTrueTrack-gps.trueTrack;
//				gps.turnRateDegSec=deltaA/deltaT;
//				gps.turnRateDegMin=gps.turnRateDegSec*60;
//				if(getMainStatus()==MAIN_DISPLAY_HSI) PrintTurnRate(gps.turnRateDegMin);
			}
			gps.trueTrack=newTrueTrack;
		} else {
			if(gps.turnRateDegSec!=0) {
				gps.turnRateDegSec=0;
				gps.turnRateDegMin=0;
//				if(getMainStatus()==MAIN_DISPLAY_HSI) PrintTurnRate(0);
			}
		}
	}
	if(gps.speedKmh>4) BlackBoxRecordCourse(newTrueTrack);
	NMEAparser.dirTimestamp=timestamp;
}

bool parseTime(char* field, int* timeHour, int* timeMin, float* timeSec) {
	if(field[0]=='\0') return false;
	return (sscanf(field,"%2d%2d%f",timeHour,timeMin,timeSec)==3); //Hour, Minute, second hhmmss.sss
}

bool parseDate(char* field, int* dd, int* mm, int* yy)
{
	if(field[0]=='\0') return false;
	if(sscanf(field,"%2d%2d%d",dd,mm,yy)!=3) return false; //Date ddmmyy
	return (*dd>0 && *mm>0);
}

bool parseLatitude(char* firstField, char* secondField, int* latDeg, float* latMin, bool* latNorth) {
	if(firstField[0]=='\0' || secondField[0]=='\0') return false;
	if(sscanf(firstField,"%2d%f",latDeg,latMin)!=2) return false; //Latitude ddmm.mm
	switch(secondField[0]) { //North or South
		case 'N':
			*latNorth=true;
			return true;
		case 'S':
			*latNorth=false;
			return true;
		default:
			return false;
	}
}

bool parseLongitude(char* firstField, char* secondField, int* lonDeg, float* lonMin, bool* lonEast) {
	if(firstField[0]=='\0' || secondField[0]=='\0') return false;
	if(sscanf(firstField,"%3d%f",lonDeg,lonMin)!=2) return false; //Longitude dddmm.mm
	return parseEastWest(secondField,lonEast);
}

bool parseValid(char* field, bool* isValid) {
	switch(field[0]) {
		case 'A':
			*isValid=true;
			return true;
		case 'V':
			*isValid=false;
			return true;
		default:
			return false;
	}
}

bool parseEastWest(char* field, bool* isEast) {
	switch(field[0]) { //East or West
		case 'E':
			*isEast=true;
			return true;
		case 'W':
			*isEast=false;
			return true;
		default:
			return false;
	}
}

bool parseInteger(char* field, int* value)
{
	if(field[0]=='\0') return false;
	return (sscanf(field,"%d",value)==1);
}

bool parseFloat(char* field, float* value)
{
	if(field[0]=='\0') return false;
	return (sscanf(field,"%f",value)==1);
}

int parseGGA() {
	if(NMEAparser.fieldId != 14) return 0;
	int timeHour, timeMin;
	float timeSec;
	if(!parseTime(NMEAparser.fields[1],&timeHour,&timeMin,&timeSec)) return -1;
	int latGra=-91, lonGra=-181;
	float latMin, lonMin;
	bool latNorth, lonEast;
	parseLatitude(NMEAparser.fields[2], NMEAparser.fields[3], &latGra, &latMin, &latNorth);
	parseLongitude(NMEAparser.fields[4], NMEAparser.fields[5], &lonGra,&lonMin, &lonEast);
	int quality=NMEAparser.fields[6][0]-'0'; //Quality
	int numOfSatellites=-1;
	parseInteger(NMEAparser.fields[7],&numOfSatellites); //Number of satellites
	float hDilutionPrecision=-1;
	parseFloat(NMEAparser.fields[8],&hDilutionPrecision); //H dilution
	float alt=-1;
	parseFloat(NMEAparser.fields[9],&alt); //Altitude
	char altUnit=NMEAparser.fields[10][0]; //Altitude unit
	float geoidalSeparation=0;
	parseFloat(NMEAparser.fields[11],&geoidalSeparation); //Geoidal separation
	char geoidalUnit=NMEAparser.fields[12][0]; //Geoidal Separation unit
	if(geoidalUnit != 'M') {
		#ifdef PRINT_SENTENCES
		printLog("WARNING: Geoidal separation unit not in meters!");
		#endif
		return 0;
	}
	float diffAge=0;
	parseFloat(NMEAparser.fields[13],&diffAge); //Age of differential GPS data
	int diffRef=-1;
	parseInteger(NMEAparser.fields[14],&diffRef); //Differential reference station ID, the last one
	float timestamp=timeHour*3600+timeMin*60+timeSec;
	if(timestamp<NMEAparser.newerTimestamp) return 0; //the sentence is old
	if(quality!=Q_NO_FIX) {
		if(timestamp>NMEAparser.newerTimestamp) { //this is a new one sentence
			NMEAparser.newerTimestamp=timestamp;
			NMEAparser.GGAfound=true;
			NMEAparser.RMCfound=false;
			NMEAparser.GSAfound=false;
		} else { //timestamp==newerTimestamp
			if(NMEAparser.GGAfound) return 0;
			else NMEAparser.GGAfound=true;
		}
		NMEAparser.alt=alt;
		NMEAparser.altUnit=altUnit;
		NMEAparser.latGra=latGra;
		NMEAparser.latMin=latMin;
		NMEAparser.latNorth=latNorth;
		NMEAparser.lonGra=lonGra;
		NMEAparser.lonMin=lonMin;
		NMEAparser.lonEast=lonEast;
		NMEAparser.timeHour=timeHour;
		NMEAparser.timeMin=timeMin;
		NMEAparser.timeSec=timeSec;
		NMEAparser.hdop=hDilutionPrecision;
		NMEAparser.SatsInView=numOfSatellites;
		updateFixMode(MODE_GPS_FIX);
		return 1;
	} else { //there is no fix...;
		updateFixMode(MODE_NO_FIX); //show that there is no fix
		if(timestamp>NMEAparser.newerTimestamp) {
			NMEAparser.newerTimestamp=timestamp;
			updateTime(timestamp,timeHour,timeMin,timeSec,true); //show the time
		}
	}
	return 0;
}

int parseRMC() {
	if(NMEAparser.fieldId!=12 && NMEAparser.fieldId!=11) return 0;
	int timeHour=-1,timeMin=-1;
	float timeSec=0;
	if(!parseTime(NMEAparser.fields[1],&timeHour,&timeMin,&timeSec)) return -1;
	bool isValid=false;
	if(!parseValid(NMEAparser.fields[2], &isValid)) return -2; //Status
	int timeDay=-1,timeMonth=-1,timeYear=-1;
	if(!parseDate(NMEAparser.fields[9],&timeDay,&timeMonth,&timeYear)) return -9; //Date
	float timestamp=timeHour*3600+timeMin*60+timeSec;
	if(timeDay!=gps.day) {
		NMEAparser.newerTimestamp=timestamp; //change of date
		NMEAparser.timeDay=timeDay;
		NMEAparser.timeMonth=timeMonth;
		NMEAparser.timeYear=timeYear;
		NMEAparser.RMCfound=isValid;
		NMEAparser.GGAfound=false;
		NMEAparser.GSAfound=false;
		updateTime(timestamp,timeHour,timeMin,timeSec,isValid); //show the time
	} else if(timestamp<NMEAparser.newerTimestamp) return 0; //the sentence is old
	else if(timestamp>NMEAparser.newerTimestamp) { //new sentence
		NMEAparser.newerTimestamp=timestamp;
		updateTime(timestamp,timeHour,timeMin,timeSec,isValid); //show the time
		NMEAparser.RMCfound=isValid;
		NMEAparser.GGAfound=false;
		NMEAparser.GSAfound=false;
	} else { // timestamp==NMEAparser.newerTimestamp
		if(NMEAparser.RMCfound) return 0;
		else NMEAparser.RMCfound=isValid;
	}
	if(isValid)
	{
		int latGra=-91, lonGra=-181;
		float latMin=0, lonMin=0;
		bool latNorth=true, lonEast=true;
		parseLatitude(NMEAparser.fields[3], NMEAparser.fields[4], &latGra, &latMin, &latNorth);
		parseLongitude(NMEAparser.fields[5], NMEAparser.fields[6], &lonGra,&lonMin, &lonEast);
		float groundSpeedKnots=0;
		parseFloat(NMEAparser.fields[7],&groundSpeedKnots); //Ground speed Knots
		float trueTrack=0;
		parseFloat(NMEAparser.fields[8],&trueTrack); //True track
		float magneticVariation=0;
		parseFloat(NMEAparser.fields[10],&magneticVariation); //Magnetic declination
		bool magneticVariationToEast=true;
		parseEastWest(NMEAparser.fields[11], &magneticVariationToEast); //Magnetic declination East or West, can be the last one
		char faa=FAA_ABSENT;
		if(NMEAparser.fieldId==12) faa=NMEAparser.fields[12][0]; //FAA Indicator (optional)
		NMEAparser.latGra=latGra;
		NMEAparser.latMin=latMin;
		NMEAparser.latNorth=latNorth;
		NMEAparser.lonGra=lonGra;
		NMEAparser.lonMin=lonMin;
		NMEAparser.lonEast=lonEast;
		NMEAparser.timeHour=timeHour;
		NMEAparser.timeMin=timeMin;
		NMEAparser.timeSec=timeSec;
		NMEAparser.groundSpeedKnots=groundSpeedKnots;
		NMEAparser.trueTrack=trueTrack;
		NMEAparser.magneticVariation=magneticVariation;
		NMEAparser.magneticVariationToEast=magneticVariationToEast;
		return 1;
	}
	return 0;
}

int parseGSA() {
	if(NMEAparser.fieldId!=17) return 0;
	bool autoSelectionMode;
	switch(NMEAparser.fields[1][0]) {  //Mode
		case 'A':
			autoSelectionMode=true;
			break;
		case 'M':
			autoSelectionMode=false;
			break;
		default:
			return (-1);
	}
	int mode=MODE_NO_FIX;
	if(!parseInteger(NMEAparser.fields[2],&mode)) return -2; //Signal Strength
	int satellites[12];
	int numOfSatellites=0;
	for(short i=0;i<12;i++) if(parseInteger(NMEAparser.fields[i+3],&satellites[i])) numOfSatellites++; //Satellites IDs
	float pdop=0;
	if(!parseFloat(NMEAparser.fields[15],&pdop)) return -15; //PDOP
	float hdop=0;
	if(!parseFloat(NMEAparser.fields[16],&hdop)) return -16; //HDOP
	float vdop=0;
	if(!parseFloat(NMEAparser.fields[17],&vdop)) return -17; //VDOP, the last one
	updateFixMode(mode);
	if(mode!=MODE_NO_FIX) {
		if(NMEAparser.GSAfound) return 0;
		else if(NMEAparser.GGAfound && NMEAparser.RMCfound) {
			NMEAparser.GSAfound=1;
			NMEAparser.pdop=pdop;
			NMEAparser.hdop=hdop;
			NMEAparser.vdop=vdop;
			NMEAparser.SatsInUse=numOfSatellites;
			return 1;
		}
		return 0;
	}
	return 0;
}

int parseGSV() {
	int numOfGSVmsg=NMEAparser.numOfGSVmsg; //take a note of numOfGSV...
	int GSVmsgSeqNo=NMEAparser.GSVmsgSeqNo; // ... and seq number
	NMEAparser.numOfGSVmsg=0; // reset "a priori"
	NMEAparser.GSVmsgSeqNo=0;
	if(NMEAparser.fieldId < 7) return 0;
	int sen,seq,sat; //Number of GSV messages, GSV message seq no, total number of satellites in view
	if(parseInteger(NMEAparser.fields[1],&sen) && parseInteger(NMEAparser.fields[2],&seq) && parseInteger(NMEAparser.fields[3],&sat)) {
		if(sen<=0 || seq<=0 || seq>sen || sat<0 || sat>MAX_NUM_SAT) return -2;
	} else return -1;
	if(seq==1) { //the first one resets GSVmsgSeqNo counter
		numOfGSVmsg=sen;
		GSVmsgSeqNo=1;
		for(int i=0; i<MAX_NUM_SAT; i++) for(int j=SAT_ELEVATION; j<=SAT_SNR; j++) gps.satellites[i][j]=-1; //reset all sats
		NMEAparser.GSVtotalSatInView=sat;
		updateNumOfTotalSatsInView(NMEAparser.GSVtotalSatInView);
	} else { //we are not expecting the first
		if(sen!=numOfGSVmsg) return -3;
		if(seq!=++GSVmsgSeqNo) return -4;
		if(sat!=NMEAparser.GSVtotalSatInView) return -5;
	}
	int pos=5;
	bool ok=true;
	while(pos<=NMEAparser.fieldId && ok) {
		int satId;
		if(!(ok=parseInteger(NMEAparser.fields[pos++],&satId))) break;
		if(!(ok=(satId<=0 || satId>MAX_NUM_SAT))) break;
		satId--;
		for(int i=SAT_ELEVATION; i<=SAT_SNR && ok && pos<=NMEAparser.fieldId; i++) {
			if(!(ok=parseInteger(NMEAparser.fields[pos++],&gps.satellites[satId][i]))) gps.satellites[satId][i]=-1;
		}
	}
	if(!ok) return(-1-pos);
	NMEAparser.numOfGSVmsg=numOfGSVmsg; //put back the right values
	NMEAparser.GSVmsgSeqNo=GSVmsgSeqNo;
	return 1;
}

unsigned int getCRCintValue(void) {
	if(NMEAparser.rcvdBytesOfSentence<9) return -1;
	char asciiCheksum[2]; //two digits
	asciiCheksum[0]=NMEAparser.sentence[NMEAparser.rcvdBytesOfSentence-2];
	asciiCheksum[1]=NMEAparser.sentence[NMEAparser.rcvdBytesOfSentence-1];
	unsigned int checksum;
	sscanf(asciiCheksum,"%x",&checksum); //read as hex
	return checksum;
}
