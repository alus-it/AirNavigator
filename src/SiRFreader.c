//============================================================================
// Name        : SiRFreader.c
// Since       : 6/7/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 6/7/2011
// Description : Reads from a SiRF serial device SiRF sentences and parse them
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
//#include <termios.h>
#include <pthread.h>
#include <time.h>
#include <sys/file.h>
#include <sys/time.h>
#include "AirNavigator.h"
#include "SiRFreader.h"

#define BUFFER_SIZE        1034
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
void processPayload(char *payload, int len, long timestamp);
void* runThread(void *ptr);

pthread_t thread;
volatile short readingSiRF=-1; //-1 means still not initialized

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
	readingSiRF=0;
	//updateNumOfTotalSatsInView(0); //Display: at the moment we have no info from GPS
	//updateNumOfActiveSats(0);
	//FBrenderFlush();
}

short SiRFreaderIsReading(void) {
	if(readingSiRF==-1) return 0; //in the case it is not initilized we are not reading..
	return readingSiRF;
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

void processPayload(char *payload, int len, long timestamp) {
	unsigned char msgID=payload[0];
	logText("Received frame ID: %d of length: %d\n",msgID,len);
	if(msgID==SIRF_GEODETIC_MSGID && len==SIRF_GEODETIC_MSG_LEN) { //we want to interpret only geodetic
		time_t curr_time,gps_time;
		struct timeval new_time;
		//char * saved_tz=NULL;
		struct geodetic_nav_data * msg=(struct geodetic_nav_data *)payload;
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
	free(payload);
}

void* runThread(void *ptr) { //listening function, it will be ran in a separate thread
	static int fd=-1;
	int maxfd;//
	fd_set readfs;//
	static unsigned char buf[BUFFER_SIZE]; //buffer for where data is put
	fd=open("/dev/gpsdata",O_RDONLY|O_NONBLOCK);
	if(fd<0) {
		fd=-1;
		//logText("ERROR: Can't open the gps serial port on device: %s\n",config.GPSdevName);
		logText("ERROR: Can't open the gps serial port on device: %s\n","/dev/gpsdata");
		readingSiRF=0;
		pthread_exit(NULL);
	}

	logText("Device opened!\n");

	maxfd=fd+1;

	//quale baudrate: {38400, 9600, 57600, 19200, 4800, 115200, 0};  ???
	//                   ???   ???   ???    ???    ???    read
	//struct termios options;
	//tcgetattr(fd, &options); // Get the current options for the port...
	//cfsetispeed(&options,B115200); // Set the baud rates to 115200...
	//options.c_cflag |= (CLOCAL | CREAD); // Enable the receiver and set local mode...
	//tcsetattr(fd, TCSANOW, &options); // Set the new options for the port...

	long payloadLength=0;
	int i,redBytes,rcvdBytesOfPayload=0,frameStatus=0,checksum=0;
	unsigned char payload[MAX_PAYLOAD_LENGHT]={0};
	unsigned char firstBytePayloadLength=0,firstByteChecksum=0;
	long timestamp;
	while(readingSiRF) { // loop while waiting for input
		FD_SET(fd,&readfs);
		select(maxfd,&readfs,NULL,NULL,NULL); //wait to read because the read is now non-blocking
		//sleep(1);
		if(readingSiRF) { //further check if we still want to read after waiting
			redBytes=read(fd,buf,BUFFER_SIZE);
			logText("Red: %d Bytes.\n",redBytes);

			timestamp=time(NULL); //get the timestamp of last char sequence received

			for(i=0;i<redBytes;i++) {
				logText("%x ",buf[i]);

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
		} //end of further if(reading) check
	} //while reading
	close(fd); //close the serial port
	pthread_exit(NULL);
	return NULL;
}

short SiRFreaderStart(void) { //function to start the listening thread
if(readingSiRF==-1) initializeSiRF();
	if(!readingSiRF) {
		readingSiRF=1;
		if(pthread_create(&thread,NULL,runThread,(void*)NULL)) {
			readingSiRF=0;
			logText("SiRFreader: ERROR unable to create the reading thread.\n");
		}
	}
	return readingSiRF;
}

void SiRFreaderStop(void) {
	if(readingSiRF==1) readingSiRF=0;
}

void SiRFreaderClose(void) {
	SiRFreaderStop();
	if(!readingSiRF) pthread_join(thread,NULL); //wait for thread death
}
