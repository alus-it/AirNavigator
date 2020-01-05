//============================================================================
// Name        : SiRFparser.h
// Since       : 6/7/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : https://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2020 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/alus-it/AirNavigator.git
// Last change : 25/11/2013
// Description : Parses SiRF messages from a GPS device
//============================================================================


#ifndef SIRFPARSER_H_
#define SIRFPARSER_H_

#define SIRF_BUFFER_SIZE        1034

void SiRFparserProcessBuffer(unsigned char *buf, long timestamp, int redBytes);


#endif
