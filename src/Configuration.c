//============================================================================
// Name        : Configuration.c
// Since       : 28/11/2012
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 24/9/2013
// Description : Implementation of Config with the shared config data struct
//============================================================================

#include <libroxml/roxml.h>
#include "Configuration.h"
#include "AirNavigator.h"
#include "TomTom.h"

void initConfig() { //Initializes the confog struct with the default values
	config.distUnit=KM;
	config.trackErrUnit=MT;
	config.speedUnit=KMH;
	config.vSpeedUnit=FTMIN;
	config.cruiseSpeed=100;
	config.stallSpeed=55; //speeds in Km/h
	config.fuelConsumption=15; //liters per hour
	config.takeOffdiffAlt=60; //meter
	config.trackErrorTolearnce=5;
	config.deptDistTolerance=1000; //meter
	config.wpDistTolerance=100; //meter
	config.courseTolerance=20; //deg
	config.sunZenith=96; //deg Civil Sun Zenith
	config.recordTimeInterval=5; //sec
	config.recordMinDist=10; //meters
	config.GPSdevName=strdup("/var/run/gpsfeed");
	config.GPSbaudRate=115200;
	config.GPSdataBits=8;
	config.GPSstopBits=1;
	config.GPSparity=0;
	config.tomtomModel=strdup("UNKNOWN");
	config.serialNumber=strdup("UNKNOWN");

	//Default color schema
	colorSchema.background=0x0000; //black
	colorSchema.compassRose=0xffff; //white
	colorSchema.dirMarker=0xf000; //red
	colorSchema.magneticDir=0x00f0; //blue
	colorSchema.routeIndicator=0x0f00; //green
	colorSchema.cdi=0xff00; //yellow
	colorSchema.cdiScale=0xffff;
	colorSchema.altScale=0xffff;
	colorSchema.vsi=0xffff;
	colorSchema.altMarker=0xffff;
	colorSchema.text=0x0f00; //green
	colorSchema.warning=0xf000; //red
}

void loadConfig() { //Load configuration
	node_t* root=roxml_load_doc("/mnt/sdcard/AirNavigator/config.xml");
	if(root!=NULL) {
		node_t *part,*detail,*attr;
		char* text=roxml_get_name(root,NULL,0);
		if(strcmp(text,"AirNavigatorConfig")==0) {
			part=roxml_get_chld(root,"aircraft",0);
			if(part!=NULL) {
				detail=roxml_get_chld(part,"speeds",0);
				if(detail!=NULL) {
					attr=roxml_get_attr(detail,"cruise",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						config.cruiseSpeed=atof(text);
					}
					attr=roxml_get_attr(detail,"Vs",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						config.stallSpeed=atof(text);
					}
				} else fprintf(logFile,"WARNING: no speeds configuration found, using default values.\n");
				detail=roxml_get_chld(part,"fuel",0);
				if(detail!=NULL) {
					attr=roxml_get_attr(detail,"consumption",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						config.fuelConsumption=atof(text);
					}
				} else fprintf(logFile,"WARNING: no fuel configuration found, using default values.\n");
			} else fprintf(logFile,"WARNING: no aircraft configuration found, using default values.\n");

			part=roxml_get_chld(root,"measureUnits",0);
			if(part!=NULL) {
				detail=roxml_get_chld(part,"speeds",0);
				if(detail!=NULL) {
					attr=roxml_get_attr(detail,"horizontal",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						if(strcmp(text,"Kmh")==0 || strcmp(text,"kmh")==0 || strcmp(text,"KMH")==0) config.speedUnit=KMH;
						else if(strcmp(text,"Knots")==0 || strcmp(text,"knots")==0) config.speedUnit=KNOTS;
							else if(strcmp(text,"MPH")==0 || strcmp(text,"mph")==0) config.speedUnit=MPH;
								else fprintf(logFile,"WARNING: Horizontal speed measure unit not recognized, using the default one.\n");
					}
					attr=roxml_get_attr(detail,"vertical",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						if(strcmp(text,"FtMin")==0) config.vSpeedUnit=FTMIN;
						else if(strcmp(text,"ms")==0) config.vSpeedUnit=MS;
							else fprintf(logFile,"WARNING: Vertical speed measure unit not recognized, using the default one.\n");
					}
				} else fprintf(logFile,"WARNING: no speeds measure units configuration found, using default values.\n");
				detail=roxml_get_chld(part,"distances",0);
				if(detail!=NULL) {
					attr=roxml_get_attr(detail,"distance",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						if(strcmp(text,"Km")==0 || strcmp(text,"km")==0) config.distUnit=KM;
						else if(strcmp(text,"NM")==0 || strcmp(text,"nm")==0) config.distUnit=NM;
							else if(strcmp(text,"Mile")==0 || strcmp(text,"mile")==0) config.distUnit=MI;
								else fprintf(logFile,"WARNING: distance measure unit not recognized, using the default one.\n");
					}
					attr=roxml_get_attr(detail,"trackError",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						if(strcmp(text,"m")==0 || strcmp(text,"Mt")==0) config.trackErrUnit=MT;
							else if(strcmp(text,"Ft")==0 || strcmp(text,"ft")==0) config.trackErrUnit=FT;
								else if(strcmp(text,"NM")==0 || strcmp(text,"nm")==0) config.trackErrUnit=NM;
									else fprintf(logFile,"WARNING: track error measure unit not recognized, using the default one.\n");
					}
				} else fprintf(logFile,"WARNING: no distances measure units configuration found, using default values.\n");
			} else fprintf(logFile,"WARNING: no speeds and distances measure units configuration found, using default values.\n");
			part=roxml_get_chld(root,"navigator",0);
			if(part!=NULL) {
				detail=roxml_get_chld(part,"takeOff",0);
				if(detail!=NULL) {
					attr=roxml_get_attr(detail,"diffAlt",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						config.takeOffdiffAlt=atof(text);
					}
				} else fprintf(logFile,"WARNING: take off configuration found, using default values.\n");
				detail=roxml_get_chld(part,"navParameters",0);
				if(detail!=NULL) {
					attr=roxml_get_attr(detail,"trackErrorTolearnce",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						config.trackErrorTolearnce=atof(text);
					}
					attr=roxml_get_attr(detail,"deptDistTolerance",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						config.deptDistTolerance=atof(text);
					}
					attr=roxml_get_attr(detail,"wpDistTolerance",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						config.wpDistTolerance=atof(text);
					}
					attr=roxml_get_attr(detail,"courseTolerance",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						config.courseTolerance=atof(text);
					}
				} else fprintf(logFile,"WARNING: no navigation parameters found, using default values.\n");
				detail=roxml_get_chld(part,"sunZenith",0);
				if(detail!=NULL) {
					attr=roxml_get_attr(detail,"angle",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						config.sunZenith=atof(text);
					}
				} else fprintf(logFile,"WARNING: no sun zenith found, using default value.\n");
			} else fprintf(logFile,"WARNING: no navigator configuration found, using default values.\n");
			part=roxml_get_chld(root,"trackRecorder",0);
			if(part!=NULL) {
				detail=roxml_get_chld(part,"update",0);
				if(detail!=NULL) {
					attr=roxml_get_attr(detail,"timeInterval",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						config.recordTimeInterval=atof(text);
					}
					attr=roxml_get_attr(detail,"distanceInterval",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						config.recordMinDist=atof(text);
					}
				} else fprintf(logFile,"WARNING: recording time and distnce intervals missing, using default values.\n");
			} else fprintf(logFile,"WARNING: no track recorder configuration found, using default values.\n");
			part=roxml_get_chld(root,"colorSchema",0);
			if(part!=NULL) {
				detail=roxml_get_chld(part,"colors",0);
				if(detail!=NULL) {
					int color;
					attr=roxml_get_attr(detail,"background",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						colorSchema.background=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"compassRose",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						colorSchema.compassRose=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"dirMarker",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						colorSchema.dirMarker=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"magneticDir",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						colorSchema.magneticDir=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"routeIndicator",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						colorSchema.routeIndicator=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"cdi",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						colorSchema.cdi=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"cdiScale",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						colorSchema.cdiScale=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"altScale",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						colorSchema.altScale=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"vsi",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						colorSchema.vsi=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"altMarker",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						colorSchema.altMarker=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"text",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						colorSchema.text=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"warning",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						colorSchema.warning=(unsigned short)color;
					}
				} else fprintf(logFile,"WARNING: in the color schema the colors are missing, using default colors.\n");
			} else fprintf(logFile,"WARNING: no color schema found, using default colors.\n");
			part=roxml_get_chld(root,"GPSreceiver",0);
			if(part!=NULL) {
				attr=roxml_get_attr(part,"devName",0);
				if(attr!=NULL) {
					text=roxml_get_content(attr,NULL,0,NULL);
					config.GPSdevName=strdup(text);
				}
				attr=roxml_get_attr(part,"baudRate",0);
				if(attr!=NULL) {
					text=roxml_get_content(attr,NULL,0,NULL);
					config.GPSbaudRate=atol(text);
				}
				attr=roxml_get_attr(part,"dataBits",0);
				if(attr!=NULL) {
					text=roxml_get_content(attr,NULL,0,NULL);
					config.GPSdataBits=atoi(text);
				}
				attr=roxml_get_attr(part,"stopBits",0);
				if(attr!=NULL) {
					text=roxml_get_content(attr,NULL,0,NULL);
					config.GPSstopBits=atoi(text);
				}
				attr=roxml_get_attr(part,"parity",0);
				if(attr!=NULL) {
					text=roxml_get_content(attr,NULL,0,NULL);
					config.GPSparity=atoi(text);
				}
			} else fprintf(logFile,"WARNING: no GPS receiver configuration found, using default values.\n");
		} else fprintf(logFile,"ERROR: configuration file config.xml with root element wrong.\n");
		roxml_release(RELEASE_ALL);
		roxml_close(root);
	} else fprintf(logFile,"WARING: configuration file not found: config.xml missing, using default settings.\n");
}