//============================================================================
// Name        : NMEAparser.h
// Since       : 9/2/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : https://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2016 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 22/11/2013
// Description : Parses NMEA sentences from a GPS device
//============================================================================

#ifndef NMEAPARSER_H_
#define NMEAPARSER_H_

#define NMEA_BUFFER_SIZE         (4096*6)
#define MAX_SENTENCE_LENGTH 255

void NMEAparserProcessBuffer(unsigned char *buf, int redBytes);



#endif
