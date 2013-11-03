//============================================================================
// Name        : Geoidal.c
// Since       : 24/1/2013
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 10/2/2013
// Description : Estimates geoidal separation from WGS86 to main sea level
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Geoidal.h"
#include "AirNavigator.h"

#define GEOID_H 19
#define GEOID_W 37

#define EGM96_H 90
#define EGM96_W 180
#define EGM96SIZE EGM96_W * EGM96_H //16200

const short geoid_data[GEOID_H][GEOID_W]={ //From: www.gliding.ch/manuels/flarm_obstacle_v3.00_en.pdf Source: US NIMA Technical Report ref: DMA TR 8350.2 Table 6.1
/*       -180,-170,...                                                            ,0,                                 ...,180   */
/*-90 */{ -30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30, -30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30}, //0
/*-80 */{ -53,-54,-55,-52,-48,-42,-38,-38,-29,-26,-26,-24,-23,-21,-19,-16,-12, -8, -4, -1,  1,  4,  4,  6,  5,  4,   2, -6,-15,-24,-33,-40,-48,-50,-53,-52,-53}, //1
/*-70 */{ -61,-60,-61,-55,-49,-44,-38,-31,-25,-16, -6,  1,  4,  5,  4,  2,  6, 12, 16, 16, 17, 21, 20, 26, 26, 22,  16, 10, -1,-16,-29,-36,-46,-55,-54,-59,-61}, //2
/*-60 */{ -45,-43,-37,-32,-30,-26,-23,-22,-16,-10, -2, 10, 20, 20, 21, 24, 22, 17, 16, 19, 25, 30, 35, 35, 33, 30,  27, 10, -2,-14,-23,-30,-33,-29,-35,-43,-45}, //3
/*-50 */{ -15,-18,-18,-16,-17,-15,-10,-10, -8, -2,  6, 14, 13,  3,  3, 10, 20, 27, 25, 26, 34, 39, 45, 45, 38, 39,  28, 13, -1,-15,-22,-22,-18,-15,-14,-10,-15}, //4
/*-40 */{  21,  6,  1, -7,-12,-12,-12,-10, -7, -1,  8, 23, 15, -2, -6,  6, 21, 24, 18, 26, 31, 33, 39, 41, 30, 24,  13, -2,-20,-32,-33,-27,-14, -2,  5, 20, 21}, //5
/*-30 */{  46, 22,  5, -2, -8,-13,-10, -7, -4,  1,  9, 32, 16,  4, -8,  4, 12, 15, 22, 27, 34, 29, 14, 15, 15,  7,  -9,-25,-37,-39,-23,-14, 15, 33, 34, 45, 46}, //6
/*-20 */{  51, 27, 10,  0, -9,-11, -5, -2, -3, -1,  9, 35, 20, -5, -6, -5,  0, 13, 17, 23, 21,  8, -9,-10,-11,-20, -40,-47,-45,-25,  5, 23, 45, 58, 57, 63, 51}, //7
/*-10 */{  36, 22, 11,  6, -1, -8,-10, -8,-11, -9,  1, 32,  4,-18,-13, -9,  4, 14, 12, 13, -2,-14,-25,-32,-38,-60, -75,-63,-26,  0, 35, 52, 68, 76, 64, 52, 36}, //8
/* 00 */{  22, 16, 17, 13,  1,-12,-23,-20,-14, -3, 14, 10,-15,-27,-18,  3, 12, 20, 18, 12,-13, -9,-28,-49,-62,-89,-102,-63, -9, 33, 58, 73, 74, 63, 50, 32, 22}, //9
/* 10 */{  13, 12, 11,  2,-11,-28,-38,-29,-10,  3,  1,-11,-41,-42,-16,  3, 17, 33, 22, 23,  2, -3, -7,-36,-59,-90, -95,-63,-24, 12, 53, 60, 58, 46, 36, 26, 13}, //10
/* 20 */{   5, 10,  7, -7,-23,-39,-47,-34, -9,-10,-20,-45,-48,-32, -9, 17, 25, 31, 31, 26, 15,  6,  1,-29,-44,-61, -67,-59,-36,-11, 21, 39, 49, 39, 22, 10,  5}, //11
/* 30 */{  -7, -5, -8,-15,-28,-40,-42,-29,-22,-26,-32,-51,-40,-17, 17, 31, 34, 44, 36, 28, 29, 17, 12,-20,-15,-40, -33,-34,-34,-28,  7, 29, 43, 20,  4, -6, -7}, //12
/* 40 */{ -12,-10,-13,-20,-31,-34,-21,-16,-26,-34,-33,-35,-26,  2, 33, 59, 52, 51, 52, 48, 35, 40, 33, -9,-28,-39, -48,-59,-50,-28,  3, 23, 37, 18, -1,-11,-12}, //13
/* 50 */{  -8,  8,  8,  1,-11,-19,-16,-18,-22,-35,-40,-26,-12, 24, 45, 63, 62, 59, 47, 48, 42, 28, 12,-10,-19,-33, -43,-42,-43,-29, -2, 17, 23, 22,  6,  2, -8}, //14
/* 60 */{   2,  9, 17, 10, 13,  1,-14,-30,-39,-46,-42,-21,  6, 29, 49, 65, 60, 57, 47, 41, 21, 18, 14,  7, -3,-22, -29,-32,-32,-26,-15, -2, 13, 17, 19,  6,  2}, //15
/* 70 */{   2,  2,  1, -1, -3, -7,-14,-24,-27,-25,-19,  3, 24, 37, 47, 60, 61, 58, 51, 43, 29, 20, 12,  5, -2,-10, -14,-12,-10,-14,-12, -6, -2,  3,  6,  4,  2}, //16
/* 80 */{   3,  1, -2, -3, -3, -3, -1,  3,  1,  5,  9, 11, 19, 27, 31, 34, 33, 34, 33, 34, 28, 23, 17, 13,  9,  4,   4,  1, -2, -2,  0,  2,  3,  2,  1,  1,  3}, //17
/* 90 */{  13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,  13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13}};//18

unsigned char* egm96data=NULL;

double interpolation2d(double x, double y, double z11, double z12, double z21, double z22) {
	return (z22*y*x+z12*(1-y)*x+z21*y*(1-x)+z11*(1-y)*(1-x)); //x and y must be between 0 and 1
}

double wgs84_to_msl_delta(double lat, double lon) { //It uses the data hard-coded in the table
	int lati=(int)((90+lat)/10);
	int loni=(int)((180+lon)/10);
	if(lati>GEOID_H || loni>GEOID_W || lati<0 || loni<0) return 0;
	if(loni==GEOID_W) loni=0;
	if(lati==GEOID_H) return ((double)geoid_data[lati-1][loni]); //If we are really at the North pole
	return(interpolation2d(
			18+lon/10-(double)loni,
			9+lat/10-(double)lati,
			(double)geoid_data[lati][loni],
			(double)geoid_data[lati][loni+1],
			(double)geoid_data[lati+1][loni],
			(double)geoid_data[lati+1][loni+1]));
}

short GeoidalOpen() {
	if(egm96data!=NULL) return 2; //The data seems already loaded do nothing
	FILE *egm96dataFile=NULL;
	egm96dataFile=fopen("/mnt/sdcard/AirNavigator/egm96s.dem","rb");
	if(egm96dataFile==NULL) {
		logText("ERROR: Unable to open the egm96 geoidal separation data file.\n");
		return 0;
	}
	unsigned long len=0;
	fseek(egm96dataFile,0,SEEK_END);
	len=ftell(egm96dataFile);
	fseek(egm96dataFile,0,SEEK_SET);
	if(len==EGM96SIZE) {
		egm96data=(unsigned char*)malloc(len+1);
		if(egm96data==NULL) {
			fclose(egm96dataFile);
			logText("ERROR: Unable to load in memory the egm96 data.\n");
			return 0;
		}
		fread(egm96data,len,1,egm96dataFile);
	} else {
		fclose(egm96dataFile);
		logText("ERROR: The egm96 geoidal separation data file has not the expected size.\n");
		return 0;
	}
	fclose(egm96dataFile);
	return 1;
}

short GeoidalIsOpen() {
	if(egm96data!=NULL) return 1;
	else return 0;
}

void GeoidalClose() {
	if(egm96data!=NULL) {
			free(egm96data);
			egm96data=NULL;
	}
}

double getEGM96data(int x, int y) {
	return (double)(egm96data[x+y*EGM96_W])-127;
}

double GeoidalGetSeparation(double lat, double lon) { //It uses the data from the EGM96 file
	if(egm96data==NULL) return wgs84_to_msl_delta(lat,lon); //If EGM96 data not available use the hard-coded data
	double y=(90.0-lat)/2.0;
	int ilat=(int)y;
	if(lon<0) lon+=360.0;
	double x=lon/2.0;
	int ilon=(int)x;
	if(ilat>EGM96_H || ilon>EGM96_W || ilat<0 || ilon<0) return 0.0;
	if(ilat==EGM96_H || ilat==EGM96_H-1) return getEGM96data(ilon,EGM96_H-1); //to prevent to go over -90
	int ilonp1;
	if(ilon!=EGM96_W-1) ilonp1=ilon+1;
	else ilonp1=0; //in this case interpolate through the Greenwich meridian
	x-=(double)ilon;
	y-=(double)ilat;
	return interpolation2d(x,y,getEGM96data(ilon,ilat),getEGM96data(ilonp1,ilat),getEGM96data(ilon,ilat+1),getEGM96data(ilonp1,ilat+1));
}
