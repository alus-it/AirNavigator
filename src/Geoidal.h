//============================================================================
// Name        : Geoidal.h
// Since       : 24/1/2013
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 10/2/2013
// Description : Header file of Geoidal.c
//============================================================================

#ifndef GEOIDAL_H_
#define GEOIDAL_H_

short GeoidalOpen(void);
short GeoidalIsOpen(void);
double wgs84_to_msl_delta(double lat, double lon);
double GeoidalGetSeparation(double lat, double lon);
void GeoidalClose(void);

#endif
