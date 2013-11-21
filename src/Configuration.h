//============================================================================
// Name        : Configuration.h
// Since       : 28/11/2012
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 21/11/2013
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
	double recordTimeInterval; //sec
	double recordMinDist; //meters
	char *GPSdevName;
	long GPSbaudRate;
	short GPSdataBits, GPSstopBits, GPSparity;
	char *tomtomModel; //model of the TomtTom device
	char *serialNumber; //TomTom device serial number ID
};

struct configuration config;

void loadConfig(void);

#endif //CONFIGURATION_H__
