//============================================================================
// Name        : SiRFparser.c
// Since       : 6/7/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 22/11/2013
// Description : Parses SiRF messages from a GPS device
//============================================================================

//TODO: This has to be completely rewritten and reviewed

//#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
//#include <time.h>
#include <sys/time.h>
#include "SiRFparser.h"
#include "AirNavigator.h"
#include "GPSreceiver.h"

#define MAX_PAYLOAD_LENGHT 1023

#define SIRF_GEODETIC_MSGID 0x29
#define SIRF_GEODETIC_MSG_LEN 91


struct geodetic_nav_data {
	uint8_t  msgID;
	uint16_t navValid;
	uint16_t navType;
	uint16_t weekNo;
	uint32_t TOW;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint16_t second;
	uint32_t satIDlist;
	int32_t latitude;
	int32_t longitude;
	uint32_t altEllips;
	uint32_t altitude;
	uint8_t mapDatum;
	uint16_t speed;
	uint16_t course;
	int16_t magneticVar;
	int16_t climbRate;
	int16_t headingRate;
	uint32_t estHposPosErr;
	uint32_t estVposPosErr;
	uint32_t estTimeErr;
	uint16_t estHvelocityErr;
	int32_t clockBias;
	uint32_t clockBiasErr;
	int32_t clockDrift;
	uint32_t clockDriftErr;
	uint32_t distance;
	uint16_t distErr;
	uint16_t headingErr;
	uint8_t SVcount;
	uint8_t HDOP;
	uint8_t addModeInfo;
};

void initializeSiRF(void);
inline uint16_t endian16_swap(uint16_t val);
inline uint32_t endian32_swap(uint32_t val);
void processPayload(char *payloadCopy, int len, long timestamp);

long payloadLength=0;
int rcvdBytesOfPayload=0,frameStatus=0,checksum=0;
unsigned char payload[MAX_PAYLOAD_LENGHT]={0};
unsigned char firstBytePayloadLength=0,firstByteChecksum=0;


//float altTimestamp=0,dirTimestamp=0,newerTimestamp=0;
//int numOfGSVmsg=0,GSVmsgSeqNo=0,GSVsatSeqNo,GSVtotalSatInView;

void initializeSiRF(void) {
	gpsSiRF.timestamp=-1;
	gpsSiRF.speedKmh=-100;
	gpsSiRF.speedKnots=-100;
	gpsSiRF.altMt=-100;
	gpsSiRF.altFt=-100;
	gpsSiRF.realAltMt=-100;
	gpsSiRF.realAltFt=-100;
	gpsSiRF.trueTrack=-500;
	gpsSiRF.day=-65;
	gpsSiRF.second=-65;
	gpsSiRF.latMinDecimal=-70;
	gpsSiRF.lonMinDecimal=-70;
	gpsSiRF.lat=100;
	gpsSiRF.pdop=50;
	gpsSiRF.hdop=50;
	gpsSiRF.vdop=50;
	gpsSiRF.fixMode=MODE_UNKNOWN;
	//updateNumOfTotalSatsInView(0); //Display: at the moment we have no info from GPS
	//updateNumOfActiveSats(0);
	//FBrenderFlush();
}

void SiRFparserProcessBuffer(unsigned char *buf, long timestamp, long redBytes) {


	logText("Red: %d Bytes.\n",redBytes);

	for(int i=0;i<redBytes;i++) {
		logText("%02X ",buf[i]); //////////////DEBUG

	switch(frameStatus) { //for each byte received in the buffer
		case 0: //waiting for start sequence
			if(buf[i]==0xA0) frameStatus=1; //found first byte of start sequence
			break;
		case 1: //waiting for second byte of sequence
			if(buf[i]==0xA2) frameStatus=2; //found second byte of start sequence
			else frameStatus=0;
			break;
		case 2: //getting the first byte of payload length
			if(buf[i]<=0x7F) { //check if it is OK //should be <=0x03 if the payload is maximum 1023??
				firstBytePayloadLength=buf[i];
				frameStatus=3;
			} else frameStatus=0;
			break;
		case 3: //getting the second byte of payload length
			payloadLength=firstBytePayloadLength*256+buf[i];
			frameStatus=4;
			rcvdBytesOfPayload=0;
			break;
		case 4: //getting bytes of the payload //TODO: implement pre-selection on msgID (first byte of payload)
			payload[rcvdBytesOfPayload++]=buf[i];
			if(rcvdBytesOfPayload==payloadLength) frameStatus=5;
			break;
		case 5: //payload finished, getting first byte of checksum
			if(buf[i]<=0x7F) { //check if it is OK
				firstByteChecksum=buf[i];
				frameStatus=6;
			} else frameStatus=0;
			break;
		case 6: //getting the second byte of checksum
			checksum=firstByteChecksum*256+buf[i];
			frameStatus=7;
			break;
		case 7: //get first byte of end sequence
			if(buf[i]==0xB0) frameStatus=8;
			else {
				frameStatus=0;
				rcvdBytesOfPayload=0;
			}
			break;
		case 8: //get second byte of end sequence
			if(buf[i]==0xB3) {
				int j, calcChecksum=0;
				for(j=0;j<payloadLength;j++) calcChecksum=(calcChecksum+payload[j])&0x7FFF;
				if(calcChecksum==checksum) { //CRC check
					char *payloadCopy=malloc(payloadLength);
					if(payloadCopy!=NULL) {
						memcpy(payloadCopy,payload,payloadLength);
						processPayload(payloadCopy,payloadLength,timestamp);
					}
				} else logText("Wrong checksum :(\n");
			}
			rcvdBytesOfPayload=0;
			frameStatus=0;
			break;
	} } logText("\n\n"); //end of for(each byte) and switch(status) of received byte
}

/** 16 bits endianess hleper */
inline uint16_t endian16_swap(uint16_t val) {
    uint16_t temp;
    temp = val & 0xFF;
    temp = (val >> 8) | (temp << 8);
    return temp;
}

/** 32 bits endianess hleper */
inline uint32_t endian32_swap(uint32_t val) {
    uint32_t temp;
    temp = ( val >> 24) | ((val & 0x00FF0000) >> 8) |  ((val & 0x0000FF00) << 8) | ((val & 0x000000FF) << 24);
    return temp;
}

void processPayload(char *payloadCopy, int len, long timestamp) {
	unsigned char msgID=payloadCopy[0];
	logText("Received frame ID: %d of length: %d\n",msgID,len);
	if(msgID==SIRF_GEODETIC_MSGID && len==SIRF_GEODETIC_MSG_LEN) { //we want to interpret only geodetic
		time_t curr_time,gps_time;
		struct timeval new_time;
		//char * saved_tz=NULL;
		struct geodetic_nav_data * msg=(struct geodetic_nav_data *)payloadCopy;
		gpsSiRF.lat_deg=((int32_t)endian32_swap(msg->latitude))/10000000;
		gpsSiRF.lat_mins=((endian32_swap(msg->latitude)*60)/10000000)%60;
		gpsSiRF.long_deg=((int32_t)endian32_swap(msg->longitude))/10000000;
		gpsSiRF.long_mins=((endian32_swap(msg->longitude)*60)/10000000)%60;
		gpsSiRF.alt_cm=endian32_swap(msg->altitude);
		gpsSiRF.sat_id_list=endian32_swap(msg->satIDlist);
		gpsSiRF.sat_nb=msg->SVcount;
		gpsSiRF.speed_kmh=(endian16_swap(msg->speed)*36)/1000;
		gpsSiRF.time.tm_sec=endian16_swap(msg->second)/1000;
		gpsSiRF.time.tm_min=msg->minute;
		gpsSiRF.time.tm_hour=msg->hour;
		gpsSiRF.time.tm_mday=msg->day;
		gpsSiRF.time.tm_mon=msg->month-1;
		gpsSiRF.time.tm_year=endian16_swap(msg->year)-1900;
		gpsSiRF.time.tm_isdst=-1;
		logText("lat: %d %d; lon: %d %d; time: %d/%d/%d %d:%d:%d\n",gpsSiRF.lat_deg,gpsSiRF.lat_mins,gpsSiRF.long_deg,gpsSiRF.long_mins,gpsSiRF.time.tm_mday,gpsSiRF.time.tm_mon,gpsSiRF.time.tm_year,gpsSiRF.time.tm_hour,gpsSiRF.time.tm_min,gpsSiRF.time.tm_sec);
		time(&curr_time);
		//saved_tz=strdup(getenv("TZ"));
		//unsetenv("TZ");
		gps_time=mktime(&gpsSiRF.time);
		if(abs(gps_time-curr_time)>10) {
			logText("Syncing clock needed ! system : %d - GPS : %d\n",(int)curr_time,(int)gps_time);
			new_time.tv_sec=gps_time;
			new_time.tv_usec=0;
			settimeofday(&new_time,NULL);
		} else logText("No need to sync.\n");
		//if(saved_tz!=NULL) {
		//	setenv("TZ",saved_tz,1);
		//	free(saved_tz);
		//}
		logText("Geodetic OK !\n");
	}
	free(payloadCopy);
}
