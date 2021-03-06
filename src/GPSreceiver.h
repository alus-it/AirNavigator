//============================================================================
// Name        : NMEAreader.h
// Since       : 9/2/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : https://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2020 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/alus-it/AirNavigator.git
// Last change : 12/12/2013
// Description : Reads from a NMEA serial device NMEA sentences and parse them
//============================================================================

#ifndef GPSRECEIVER_H_
#define GPSRECEIVER_H_

#include <pthread.h>
#include "Common.h"

#define MAX_NUM_SAT 24

//FAA Mode Indicators
#define FAA_ABSENT  0 //Previous version of NMEA 2.3 without FAA
#define FAA_AUTO   65 //'A' Autonomous mode
#define FAA_DIFF   68 //'D' Differential Mode
#define FAA_ESTIM  69 //'E' Estimated (dead-reckoning) mode
#define FAA_MANUAL 77 //'M' Manual Input Mode
#define FAA_SIMUL  83 //'S' Simulated Mode
#define FAA_NOTVAL 78 //'N' Data Not Valid

enum qualityIndicators { //Quality indicators
	Q_NO_FIX,   //fix not available,
	Q_GPS_FIX,  //GPS fix,
	Q_DIFF_FIX, //Differential GPS fix
	Q_PPS_FIX,  //PPS fix
	Q_RTK_FIX,  //Real Time Kinematic
	Q_FRTK_FIX, //Float RTK
	Q_EST_FIX,  //estimated (dead reckoning)
	Q_MAN_FIX,  //Manual input mode
	Q_SIM_FIX   //Simulation mode
};

enum GPSmode {
	MODE_UNKNOWN,
	MODE_NO_FIX,
	MODE_2D_FIX,
	MODE_3D_FIX,
	MODE_GPS_FIX //This is the fix from NMEA GGA sentence: we don't know if it is 2D or 3D
};

enum indexSatData { //Indexes for satellite's data
	SAT_ELEVATION, //elevation in degrees (00-90)
	SAT_AZIMUTH,   //azimuth in degrees to true north (000-359)
	SAT_SNR        //SNR in dB (00-99)
};

struct GPSdata {
	float timestamp;                               //timestamp of the data in sec from the beginning of the day
	double speedKmh,speedKnots;                    //ground speeds in Km/h and knots
	double altMt,altFt;                            //altitudes in m and feet respect WGS84 as received by the GPS
	double realAltMt,realAltFt;                    //altitudes in m and feet respect m.s.l.
	double climbFtMin;                             //calculated vertical speed rate in feet/min
	double trueTrack,magneticTrack;                //true and magnetic track in deg
	double magneticVariation;                      //magnetic variation in deg
	bool isMagVarToEast;                           // true if magnetic variation is to east
	double turnRateDegMin,turnRateDegSec;          //calculated turn rate in deg/min and deg/sec
	int day,month,year;                            //date
	int hour,minute;                               //UTC time
	float second;                                  //seconds of UTC time
	int latDeg,lonDeg;                             //deg part of latitude and longitude
	float latMinDecimal,lonMinDecimal;             //decimal minutes and seconds part of lat and lon
	bool isLatN,isLonE;                            //true if latitude is to North, true if longitude is to East
	double lat,lon;                                //latitude and longitude expressed in rad
	float pdop,hdop,vdop;                          //P,H,V dilutions in m
	char activeSats,satsInView;                    //used and visible sats
	enum GPSmode fixMode;                          //type of fix
	int signalStrength,SNR,beaconDataRate,channel; //data about GPS signal (not used)
	int beaconFrequency;                           //beacon frequency of GPS signal (not used)
	int satellites[MAX_NUM_SAT][3];                //matrix of detected satellites
	pthread_mutex_t mutex;                         //mutex for reading and writing GPS data in this struct
};

struct GPSdata gps;

char GPSreceiverStart(void);
void GPSreceiverStop(void);
void GPSreceiverClose(void);

char updateDate(int newDay, int newMonth, int newYear);
void updateTime(float timestamp, int newHour, int newMin, float newSec, bool timeWithNoFix);
void updateGroundSpeedAndDirection(float newSpeedKmh, float newSpeedKnots, float newTrueTrack, float newMagneticTrack);
void updateSpeed(float newSpeedKnots);
void updateNumOfTotalSatsInView(int totalSats);
void updateNumOfActiveSats(int workingSats);
void updateFixMode(int fixMode);
//void updateHdiluition(float hDiluition);
//void updateDiluition(float pDiluition, float hDiluition, float vDiluition);

#endif /* GPSRECEIVER_H_ */
