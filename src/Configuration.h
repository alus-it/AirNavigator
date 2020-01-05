//============================================================================
// Name        : Configuration.h
// Since       : 28/11/2012
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : https://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2016 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/alus-it/AirNavigator.git
// Last change : 2/8/2014
// Description : Header of Config with the shared config data struct
//============================================================================

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

enum lengthMeasureUnit {
	KM,  //Distance in Kilometers
	NM,  //Distance in Nautical Miles
	MI,  //Distance in Imperial Miles
	MT,  //Length in Meters
	FT  //Length in Feet
};

enum speedMesureUnit {
	KMH,   //Speed in Km/h
	KNOTS, //Speed in Knots
	MPH,   //Speed in Miles per Hour
	FTMIN, //Vertical speed in Feet each Minute
	MS     //Vertical speed in Meters each Second
};

struct colorConfig {
	unsigned short background;
	unsigned short compassRose;
	unsigned short dirMarker;
	unsigned short magneticDir;
	unsigned short routeIndicator;
	unsigned short cdi;
	unsigned short cdiScale;
	unsigned short bearing;
	unsigned short altScale;
	unsigned short vsi;
	unsigned short altMarker;
	unsigned short text;
	unsigned short ok;
	unsigned short warning;
	unsigned short caution;
	unsigned short airplaneSymbol;
	unsigned short buttonEnabled;
	unsigned short buttonDisabled;
	unsigned short buttonLabelEnabled;
	unsigned short buttonLabelDisabled;
};

struct configuration {
	enum lengthMeasureUnit distUnit;
	enum lengthMeasureUnit trackErrUnit;
	enum speedMesureUnit speedUnit;
	enum speedMesureUnit vSpeedUnit;
	double cruiseSpeed,stallSpeed; //speeds in Km/h
	double fuelConsumption; //liters per hour
	double takeOffdiffAlt; //meters
	double trackErrorTolearnce, deptDistTolerance; //meters
	double sunZenith; //deg
	int timeZone; //local time offset from UTC time in hours
	double recordTimeInterval; //sec
	double recordMinDist; //meters
	char *GPSdevName;
	long GPSbaudRate;
	short GPSdataBits, GPSstopBits, GPSparity;
	char *tomtomModel; //model of the TomtTom device
	char *serialNumber; //TomTom device serial number ID
	struct colorConfig colorSchema;
};

struct configuration config;

void loadConfig(void);

#endif //CONFIGURATION_H__
