//============================================================================
// Name        : SiRFparser.c
// Since       : 6/7/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : https://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2020 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/alus-it/AirNavigator.git
// Last change : 27/11/2013
// Description : Parses SiRF messages from a GPS device
//============================================================================

//TODO: This has to be finished and reviewed

#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdint.h>
//#include <time.h>
//#include <sys/time.h>
#include "SiRFparser.h"
#include "Common.h"
#include "GPSreceiver.h"

#define MAX_PAYLOAD_LENGHT 1023

#define SIRF_GEODETIC_MSGID 0x29
#define SIRF_GEODETIC_MSG_LEN 91


enum SiRFparserStatus {
	SIRF_START_SEQ_1,   //waiting for start sequence
	SIRF_START_SEQ_2,   //waiting for second byte of sequence
	SIRF_PAYLOAD_LEN_1, //getting the first byte of payload length
	SIRF_PAYLOAD_LEN_2, //getting the second byte of payload length
	SIRF_PAYLOAD,       //getting bytes of the payload
	SIRF_CHECKSUM_1,    //getting first byte of checksum
	SIRF_CHECKSUM_2,    //getting the second byte of checksum
	SIRF_END_SEQ_1,     //get first byte of end sequence
	SIRF_END_SEQ_2      //get second byte of end sequence
};

static struct SiRFparserStruct {
	enum SiRFparserStatus frameStatus;
	int payloadLength,rcvdBytesOfPayload,checksum;
	unsigned char payload[MAX_PAYLOAD_LENGHT];
	unsigned char firstBytePayloadLength,firstByteChecksum;
};

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

inline uint16_t endian16_swap(uint16_t val);
inline uint32_t endian32_swap(uint32_t val);
void processPayload(char *payloadCopy, int len, long timestamp);

static struct SiRFparserStruct SiRFparser = {
	.frameStatus=SIRF_START_SEQ_1,
	.payloadLength=0,
	.rcvdBytesOfPayload=0,
	.checksum=0,
	.firstBytePayloadLength=0,
	.firstByteChecksum=0
};


void SiRFparserProcessBuffer(unsigned char *buf, long timestamp, int redBytes) {
	for(int i=0;i<redBytes;i++) {
		printLog("%02X ",buf[i]); //////////////DEBUG

	switch(SiRFparser.frameStatus) { //for each byte received in the buffer
		case SIRF_START_SEQ_1: //waiting for start sequence
			if(buf[i]==0xA0) SiRFparser.frameStatus=SIRF_START_SEQ_2; //found first byte of start sequence
			break;
		case SIRF_START_SEQ_2: //waiting for second byte of sequence
			if(buf[i]==0xA2) SiRFparser.frameStatus=SIRF_PAYLOAD_LEN_1; //found second byte of start sequence
			else SiRFparser.frameStatus=SIRF_START_SEQ_1;
			break;
		case SIRF_PAYLOAD_LEN_1: //getting the first byte of payload length
			if(buf[i]<=0x7F) { //check if it is OK //should be <=0x03 if the payload is maximum 1023??
				SiRFparser.firstBytePayloadLength=buf[i];
				SiRFparser.frameStatus=SIRF_PAYLOAD_LEN_2;
			} else SiRFparser.frameStatus=SIRF_START_SEQ_1;
			break;
		case SIRF_PAYLOAD_LEN_2: //getting the second byte of payload length
			SiRFparser.payloadLength=SiRFparser.firstBytePayloadLength*256+buf[i];
			SiRFparser.frameStatus=SIRF_PAYLOAD;
			SiRFparser.rcvdBytesOfPayload=0;
			break;
		case SIRF_PAYLOAD: //getting bytes of the payload //TODO: implement pre-selection on msgID (first byte of payload)
			SiRFparser.payload[SiRFparser.rcvdBytesOfPayload++]=buf[i];
			if(SiRFparser.rcvdBytesOfPayload==SiRFparser.payloadLength) SiRFparser.frameStatus=SIRF_CHECKSUM_1;
			break;
		case SIRF_CHECKSUM_1: //payload finished, getting first byte of checksum
			if(buf[i]<=0x7F) { //check if it is OK
				SiRFparser.firstByteChecksum=buf[i];
				SiRFparser.frameStatus=SIRF_CHECKSUM_2;
			} else SiRFparser.frameStatus=SIRF_START_SEQ_1;
			break;
		case SIRF_CHECKSUM_2: //getting the second byte of checksum
			SiRFparser.checksum=SiRFparser.firstByteChecksum*256+buf[i];
			SiRFparser.frameStatus=SIRF_END_SEQ_1;
			break;
		case SIRF_END_SEQ_1: //get first byte of end sequence
			if(buf[i]==0xB0) SiRFparser.frameStatus=SIRF_END_SEQ_2;
			else {
				SiRFparser.frameStatus=SIRF_START_SEQ_1;
				SiRFparser.rcvdBytesOfPayload=0;
			}
			break;
		case SIRF_END_SEQ_2: //get second byte of end sequence
			if(buf[i]==0xB3) {
				int j, calcChecksum=0;
				for(j=0;j<SiRFparser.payloadLength;j++) calcChecksum=(calcChecksum+SiRFparser.payload[j])&0x7FFF;
				if(calcChecksum==SiRFparser.checksum) { //CRC check
					char *payloadCopy=malloc(SiRFparser.payloadLength);
					if(payloadCopy!=NULL) {
						memcpy(payloadCopy,SiRFparser.payload,SiRFparser.payloadLength);
						processPayload(payloadCopy,SiRFparser.payloadLength,timestamp);
					}
				} else printLog("Wrong checksum :(\n");
			}
			SiRFparser.rcvdBytesOfPayload=0;
			SiRFparser.frameStatus=SIRF_START_SEQ_1;
			break;
		default:
			break;
	} } printLog("\n\n"); //end of for(each byte) and switch(status) of received byte
}

void processPayload(char *payloadCopy, int len, long timestamp) {
	unsigned char msgID=payloadCopy[0];
	printLog("Received frame ID: %d of length: %d\n",msgID,len);
	if(msgID==SIRF_GEODETIC_MSGID && len==SIRF_GEODETIC_MSG_LEN) { //we want to interpret only geodetic
//TODO: put the data in the right place: in the gps struct of GPSreceiver
//		time_t curr_time,gps_time;
//		struct timeval new_time;
//		//char * saved_tz=NULL;
//		struct geodetic_nav_data * msg=(struct geodetic_nav_data *)payloadCopy;
//		lat_deg=((int32_t)endian32_swap(msg->latitude))/10000000;
//		lat_mins=((endian32_swap(msg->latitude)*60)/10000000)%60;
//		long_deg=((int32_t)endian32_swap(msg->longitude))/10000000;
//		long_mins=((endian32_swap(msg->longitude)*60)/10000000)%60;
//		alt_cm=endian32_swap(msg->altitude);
//		sat_id_list=endian32_swap(msg->satIDlist);
//		sat_nb=msg->SVcount;
//		speed_kmh=(endian16_swap(msg->speed)*36)/1000;
//		time.tm_sec=endian16_swap(msg->second)/1000;
//		time.tm_min=msg->minute;
//		time.tm_hour=msg->hour;
//		time.tm_mday=msg->day;
//		time.tm_mon=msg->month-1;
//		time.tm_year=endian16_swap(msg->year)-1900;
//		time.tm_isdst=-1;
//		printLog("lat: %d %d; lon: %d %d; time: %d/%d/%d %d:%d:%d\n",lat_deg,lat_mins,long_deg,long_mins,time.tm_mday,time.tm_mon,time.tm_year,time.tm_hour,time.tm_min,time.tm_sec);
//		time(&curr_time);
//		//saved_tz=strdup(getenv("TZ"));
//		//unsetenv("TZ");
//		gps_time=mktime(&time);
//		if(abs(gps_time-curr_time)>10) {
//			printLog("Syncing clock needed ! system : %d - GPS : %d\n",(int)curr_time,(int)gps_time);
//			new_time.tv_sec=gps_time;
//			new_time.tv_usec=0;
//			settimeofday(&new_time,NULL);
//		} else printLog("No need to sync.\n");
//		//if(saved_tz!=NULL) {
//		//	setenv("TZ",saved_tz,1);
//		//	free(saved_tz);
//		//}
		printLog("Geodetic OK !\n");
	}
	free(payloadCopy);
}

inline uint16_t endian16_swap(uint16_t val) { //16 bits endianess helper
	uint16_t temp;
	temp = val & 0xFF;
	temp = (val >> 8) | (temp << 8);
	return temp;
}

inline uint32_t endian32_swap(uint32_t val) { //32 bits endianess hleper
	uint32_t temp;
	temp = ( val >> 24) | ((val & 0x00FF0000) >> 8) |  ((val & 0x0000FF00) << 8) | ((val & 0x000000FF) << 24);
	return temp;
}
