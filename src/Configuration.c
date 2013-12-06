//============================================================================
// Name        : Configuration.c
// Since       : 28/11/2012
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 06/12/2013
// Description : Implementation of Config with the shared config data struct
//============================================================================

#define _GNU_SOURCE

#include <stdio.h>
#include <libroxml/roxml.h>
#include "Configuration.h"
#include "Common.h"
#include "FBrender.h"

struct configuration config = { //Default configuration values
	.distUnit=KM,
	.trackErrUnit=MT,
	.speedUnit=KMH,
	.vSpeedUnit=FTMIN,
	.cruiseSpeed=100,
	.stallSpeed=55, //speeds in Km/h
	.fuelConsumption=15, //liters per hour
	.takeOffdiffAlt=60, //meter
	.trackErrorTolearnce=5,
	.deptDistTolerance=1000, //meter
	.sunZenith=96, //deg Civil Sun Zenith
	.recordTimeInterval=5, //sec
	.recordMinDist=10, //meters
	.GPSdevName=NULL,
	.GPSbaudRate=115200,
	.GPSdataBits=8,
	.GPSstopBits=1,
	.GPSparity=0,
	.tomtomModel=NULL,
	.serialNumber=NULL,
	.colorSchema = {       //Default colors
		.background=0x0000,     //black
		.compassRose=0xffff,    //white
		.dirMarker=0xf000,      //red
		.magneticDir=0x00f0,    //blue
		.routeIndicator=0x0f00, //green
		.cdi=0xff00,            //yellow
		.cdiScale=0xffff,       //white
		.altScale=0xffff,       //white
		.vsi=0xffff,            //white
		.altMarker=0xffff,      //white
		.text=0x0f00,           //green
		.ok=0x0f00,             //green
		.warning=0xff00,        //yellow
		.caution=0xf000,        //red
		.airplaneSymbol=0xfa00  //orange
	}
};

void loadConfig(void) { //Load configuration
	char *configPath;
	asprintf(&configPath,"%sconfig.xml",BASE_PATH);
	node_t* root=roxml_load_doc(configPath);
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
				} else printLog("WARNING: no speeds configuration found, using default values.\n");
				detail=roxml_get_chld(part,"fuel",0);
				if(detail!=NULL) {
					attr=roxml_get_attr(detail,"consumption",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						config.fuelConsumption=atof(text);
					}
				} else printLog("WARNING: no fuel configuration found, using default values.\n");
			} else printLog("WARNING: no aircraft configuration found, using default values.\n");
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
								else printLog("WARNING: Horizontal speed measure unit not recognized, using the default one.\n");
					}
					attr=roxml_get_attr(detail,"vertical",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						if(strcmp(text,"FtMin")==0) config.vSpeedUnit=FTMIN;
						else if(strcmp(text,"ms")==0) config.vSpeedUnit=MS;
							else printLog("WARNING: Vertical speed measure unit not recognized, using the default one.\n");
					}
				} else printLog("WARNING: no speeds measure units configuration found, using default values.\n");
				detail=roxml_get_chld(part,"distances",0);
				if(detail!=NULL) {
					attr=roxml_get_attr(detail,"distance",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						if(strcmp(text,"Km")==0 || strcmp(text,"km")==0) config.distUnit=KM;
						else if(strcmp(text,"NM")==0 || strcmp(text,"nm")==0) config.distUnit=NM;
							else if(strcmp(text,"Mile")==0 || strcmp(text,"mile")==0) config.distUnit=MI;
								else printLog("WARNING: distance measure unit not recognized, using the default one.\n");
					}
					attr=roxml_get_attr(detail,"trackError",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						if(strcmp(text,"m")==0 || strcmp(text,"Mt")==0) config.trackErrUnit=MT;
							else if(strcmp(text,"Ft")==0 || strcmp(text,"ft")==0) config.trackErrUnit=FT;
								else if(strcmp(text,"NM")==0 || strcmp(text,"nm")==0) config.trackErrUnit=NM;
									else printLog("WARNING: track error measure unit not recognized, using the default one.\n");
					}
				} else printLog("WARNING: no distances measure units configuration found, using default values.\n");
			} else printLog("WARNING: no speeds and distances measure units configuration found, using default values.\n");
			part=roxml_get_chld(root,"navigator",0);
			if(part!=NULL) {
				detail=roxml_get_chld(part,"takeOff",0);
				if(detail!=NULL) {
					attr=roxml_get_attr(detail,"diffAlt",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						config.takeOffdiffAlt=atof(text);
					}
				} else printLog("WARNING: take off configuration found, using default values.\n");
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
				} else printLog("WARNING: no navigation parameters found, using default values.\n");
				detail=roxml_get_chld(part,"sunZenith",0);
				if(detail!=NULL) {
					attr=roxml_get_attr(detail,"angle",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						config.sunZenith=atof(text);
					}
				} else printLog("WARNING: no sun zenith found, using default value.\n");
			} else printLog("WARNING: no navigator configuration found, using default values.\n");
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
				} else printLog("WARNING: recording time and distnce intervals missing, using default values.\n");
			} else printLog("WARNING: no track recorder configuration found, using default values.\n");
			part=roxml_get_chld(root,"colorSchema",0);
			if(part!=NULL) {
				detail=roxml_get_chld(part,"colors",0);
				if(detail!=NULL) {
					unsigned int color;
					attr=roxml_get_attr(detail,"background",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						config.colorSchema.background=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"compassRose",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						config.colorSchema.compassRose=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"dirMarker",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						config.colorSchema.dirMarker=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"magneticDir",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						config.colorSchema.magneticDir=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"routeIndicator",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						config.colorSchema.routeIndicator=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"cdi",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						config.colorSchema.cdi=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"cdiScale",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						config.colorSchema.cdiScale=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"altScale",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						config.colorSchema.altScale=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"vsi",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						config.colorSchema.vsi=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"altMarker",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						config.colorSchema.altMarker=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"text",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						config.colorSchema.text=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"ok",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						config.colorSchema.ok=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"warning",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						config.colorSchema.warning=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"caution",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						config.colorSchema.caution=(unsigned short)color;
					}
					attr=roxml_get_attr(detail,"airplaneSymbol",0);
					if(attr!=NULL) {
						text=roxml_get_content(attr,NULL,0,NULL);
						sscanf(text,"%x",&color);
						config.colorSchema.airplaneSymbol=(unsigned short)color;
					}
				} else printLog("WARNING: in the color schema the colors are missing, using default colors.\n");
			} else printLog("WARNING: no color schema found, using default colors.\n");
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
			} else printLog("WARNING: no GPS receiver configuration found, using default values.\n");
		} else printLog("ERROR: configuration file config.xml with root element wrong.\n");
		roxml_release(RELEASE_ALL);
		roxml_close(root);
	} else printLog("WARNING: configuration file not found: config.xml missing, using default settings.\n");
	free(configPath);
}
