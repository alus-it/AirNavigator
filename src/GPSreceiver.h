//============================================================================
// Name        : NMEAreader.h
// Since       : 9/2/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 11/11/2013
// Description : Reads from a NMEA serial device NMEA sentences and parse them
//============================================================================

#ifndef GPSRECEIVER_H_
#define GPSRECEIVER_H_

//Quality indicators
#define Q_NO_FIX   0 //fix not available,
#define Q_GPS_FIX  1 //GPS fix,
#define Q_DIFF_FIX 2 //Differential GPS fix
#define Q_PPS_FIX  3 //PPS fix
#define Q_RTK_FIX  4 //Real Time Kinematic
#define Q_FRTK_FIX 5 //Float RTK
#define Q_EST_FIX  6 //estimated (dead reckoning)
#define Q_MAN_FIX  7 //Manual input mode
#define Q_SIM_FIX  8 //Simulation mode

//FAA Mode Indicators
#define FAA_ABSENT  0 //Previous version of NMEA 2.3 without FAA
#define FAA_AUTO   65 //'A' Autonomous mode
#define FAA_DIFF   68 //'D' Differential Mode
#define FAA_ESTIM  69 //'E' Estimated (dead-reckoning) mode
#define FAA_MANUAL 77 //'M' Manual Input Mode
#define FAA_SIMUL  83 //'S' Simulated Mode
#define FAA_NOTVAL 78 //'N' Data Not Valid

#define MODE_UNKNOWN 0
#define MODE_NO_FIX  1
#define MODE_2D_FIX  2
#define MODE_3D_FIX  3
#define MODE_GPS_FIX 4 //This is the fix from GGA we don't know if it is 2D or 3D

#define MAX_NUM_SAT 24

//Indexes for satellite's data
#define SAT_PRN       0  //satellite PRN number
#define SAT_ELEVATION 1  //elevation in degrees (00-90)
#define SAT_AZIMUTH   2  //azimuth in degrees to true north (000-359)
#define SAT_SNR       3  //SNR in dB (00-99)

struct GPSdata {
	float timestamp; //timestamp of the data in sec from the beginning of the day
	double speedKmh,speedKnots; //ground speeds in Km/h and knots
	double altMt,altFt; //altitudes in m and feet respect WGS84 as received by the GPS
	double realAltMt,realAltFt; //altitudes in m and feet respect m.s.l.
	double climbFtMin; //calculated vertical speed rate in feet/min
	double trueTrack,magneticTrack; //true and magnetc track in deg
	double magneticVariation; //magnetic variation in deg
	char isMagVarToEast; // =1 (true) if magnetic variation is to east
	double turnRateDegMin,turnRateDegSec; //calculated turn rate in deg/min and deg/sec
	int day,month,year; //date
	int hour,minute; //UTC time
	float second; //seconds of UTC time
	int latDeg,lonDeg; //deg part of latitude and longitude
	float latMinDecimal,lonMinDecimal; //decimal minutes and seconds part of lat and lon
	char isLatN,isLonE; //if latitude is to North, if longitude is to East
	double lat,lon; //latitude and longitude expressed in rad
	float pdop,hdop,vdop; //P,H,V dilutions in m
	char activeSats,satsInView,fixMode; //used and visible sats, type of fix
	int signalStrength,SNR,beaconDataRate,channel; //data about GPS signal (not used)
	int beaconFrequency; //beacon frequency of GPS signal (not used)
	int satellites[MAX_NUM_SAT][4]; //matrix of detected satellites
};

struct GPSdata gps;

char GPSreceiverStart(void);
void GPSreceiverStop(void);
void GPSreceiverClose(void);

char updateDate(int newDay, int newMonth, int newYear);
void updateTime(float timestamp, int newHour, int newMin, float newSec, char timeWithNoFix);
void updateGroundSpeedAndDirection(float newSpeedKmh, float newSpeedKnots, float newTrueTrack, float newMagneticTrack);
void updateSpeed(float newSpeedKnots);
void updateNumOfTotalSatsInView(int totalSats);
void updateNumOfActiveSats(int workingSats);
void updateFixMode(int fixMode);
void updateHdiluition(float hDiluition);
void updateDiluition(float pDiluition, float hDiluition, float vDiluition);

#endif /* GPSRECEIVER_H_ */
