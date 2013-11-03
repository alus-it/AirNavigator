//============================================================================
// Name        : main.c
// Since       : 8/7/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 2/11/2013
// Description : main() function of the program for TomTom devices
//============================================================================

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <dirent.h>
#include <fcntl.h>
#include <pthread.h>
#include "AirNavigator.h"
#include "Configuration.h"
#include "TomTom.h"
#include "NMEAreader.h"
#include "Navigator.h"
#include "AirCalc.h"
#include "BlackBox.h"
#include "HSI.h"
//#include "SiRFreader.h"
//#include "tpgps.h"
#include "Geoidal.h"

#ifndef VERSION
#define VERSION "0.2.6"
#endif

//TODO: a common base path predefined: /mnt/sdcard/AirNavigator/

typedef struct fileName {
	int seqNo;             //sequence number
	char *name;            //name of the file
	struct fileName *prev; //Pointer to the previous filename in the list
	struct fileName *next; //Pointer to the next filename in the list
} *fileEntry;

pthread_mutex_t logMutex=PTHREAD_MUTEX_INITIALIZER;
FILE *logFile;

int logText(const char *texts, ...) {
	int done=-1;
	if(logFile!=NULL) {
		va_list arg;
		va_start(arg,texts);
		pthread_mutex_lock(&logMutex);
		done=vfprintf(logFile,texts,arg);
		pthread_mutex_unlock(&logMutex);
   	va_end(arg);
   	return done;
	}
	return done;
}

int main(int argc, char** argv) {
	logFile=NULL;
	if(!FbRender_Open()) exit(EXIT_FAILURE);
	fileEntry fileList=NULL, currFile=NULL; //the list of the found GPX flight plans and the pointer to the current one

	int numGPXfiles=0;
	int x=0,y=0; //coordinates of the touch screen

	//Initialization
	TsScreen_Init();
	FbRender_Flush();

	initConfig(); //initialize the configuration with the default values

	FbRender_Flush();
	{ // flush pen input
		int x=0,y=0,pen=0;
		while(TsScreen_pen(&x,&y,&pen));
	}

	logFile=fopen("/mnt/sdcard/AirNavigator/log.txt","w"); //log file
	if(logFile==NULL) {
		FbRender_BlitText(5,60,colorSchema.warning,colorSchema.background,0,"ERROR: Unable to create the logFile file!");
		FbRender_Flush();
	}
	logText("AirNavigator v. %s - Compiled: %s %s - www.alus.it\n",VERSION,__DATE__,__TIME__);


	//FIXME: Check battery status (not working)
	/*short batVolt, refVolt, chargeCurr;
	 if(checkBattery(&batVolt,&refVolt,&chargeCurr)) {
	 logText("Battery Voltage = %d\n",batVolt);           // battery voltage
	 logText("Reference Voltage = %d\n",refVolt);       // reference voltage
	 logText("Charge Current = %d\n",chargeCurr);             // Charge current
	 } else logText("ERROR: unable to get the battery status.\n");*/

	//Read TomTom model
	FILE *fd=NULL;
	fd=fopen("/proc/barcelona/modelname","r");
	if(fd==NULL) logText("ERROR: unable to read TomTom device model name.\n");
	else {
		char buf[30];
		char *ret=fgets(buf,29,fd);
		if(ret==buf) {
			free(config.tomtomModel);
			if(buf[strlen(buf)-1]=='\n') buf[strlen(buf)-1]='\0';
			config.tomtomModel=strdup(buf);
			logText("TomTom model name: %s\n",config.tomtomModel);
		} else logText("ERROR: unable to read TomTom device model name.\n");
		fclose(fd);
	}

	//Read serial number serial number
	fd=NULL;
	fd=fopen("/mnt/flash/sysfile/id","r");
	if(fd==NULL) logText("ERROR: unable to read TomTom device serial number ID.\n");
	else {
		char buf[30];
		char *ret=fgets(buf,29,fd);
		if(ret==buf) {
			free(config.serialNumber);
			config.serialNumber=strdup(buf);
			logText("TomTom device serial number ID: %s\n",config.serialNumber);
		} else logText("ERROR: unable to read TomTom device serial number ID.\n");
		fclose(fd);
	}
	logText("Screen resolution: %dx%d pixel\n",screen.width,screen.height); //logFile screen resolution

	//This is to draw some rainbow lines to put where we think it's going to crash in order to understand where it crashes
	//DrawTwoPointsLine(10,10,200,200,0xF000);
	//DrawTwoPointsLine(20,10,210,200,0xFF00);
	//DrawTwoPointsLine(30,10,220,200,0x0F00);
	//DrawTwoPointsLine(40,10,230,200,0x00F0);
	//FbRender_Flush();

	//Load configuration
	loadConfig();
	logText("Configuration loaded.\n");

	//Prepare the list of available flight plans found in the routes folder
	struct dirent *entry;
	DIR *dir = opendir("/mnt/sdcard/AirNavigator/Routes");
	if(dir!=NULL) {
		int len;
		while((entry=readdir(dir))!=NULL) { //make the list of GPX files...
			len=strlen(entry->d_name);
			if(len>3)
				if( (entry->d_name[len-3]=='G' || entry->d_name[len-3]=='g') &&
						(entry->d_name[len-2]=='P' || entry->d_name[len-2]=='p') &&
						(entry->d_name[len-1]=='X' || entry->d_name[len-1]=='x')) {
					fileEntry newFile=(fileEntry)malloc(sizeof(struct fileName));
					newFile->name=strdup(entry->d_name);
					newFile->seqNo=numGPXfiles;
					newFile->next=NULL;
					if(numGPXfiles==0) { //it's the first
							fileList=newFile;
							newFile->prev=NULL;
					} else {
							currFile->next=newFile;
							newFile->prev=currFile; //link to the previous
					}
					currFile=newFile;
					numGPXfiles++;
				}
		}
		closedir(dir);
		if(numGPXfiles==0) logText("WARNING: No GPX flight plans found.\n");
	} else logText("ERROR: could not open the Routes directory.\n");
	logText("List of GPX files created.\n");

	//Display a "menu" with the list of GPX files
	char *toLoad=NULL; //the path to the chosen GPX file to be loaded
	if(numGPXfiles>0) {
		currFile=fileList;
		short DoLoad=0;
		FbRender_BlitText(100,20,colorSchema.dirMarker,colorSchema.background,0,"AirNavigator");
		FbRender_BlitText(100,30,colorSchema.magneticDir,colorSchema.background,0,"http://www.alus.it/airnavigator");
		FbRender_BlitText(20,60,colorSchema.text,colorSchema.background,0,"Select a GPX flight plan:");
		FbRender_BlitText(200,240,colorSchema.text,colorSchema.background,0,"LOAD");
		while(!DoLoad) {
			FbRender_BlitText(20,70,colorSchema.cdi,colorSchema.background,0,"%s                                         ",currFile->name); //print the name of the current file
			if(currFile->prev!=NULL) FbRender_BlitText(20,240,colorSchema.text,colorSchema.background,0,"<< Prev");
			else FbRender_BlitText(20,240,colorSchema.text,colorSchema.background,0,"       ");
			if(currFile->next!=NULL) FbRender_BlitText(350,240,colorSchema.text,colorSchema.background,0,"Next >>");
			else FbRender_BlitText(350,240,colorSchema.text,colorSchema.background,0,"       ");
			FbRender_Flush();
			while(!TsScreen_touch(&x,&y)); //wait user presses a "button"
			if(y>200) {
				if(x<80 && currFile->prev!=NULL) currFile=currFile->prev;
				else if(x>200 && x<300) {
					asprintf(&toLoad,"%s%s","/mnt/sdcard/AirNavigator/Routes/",currFile->name);
					DoLoad=1;
				} else if(x>350 && currFile->next!=NULL) currFile=currFile->next;
			}
		}
	}

	FbRender_Clear(0,screen.height,colorSchema.background); //draw the main screen
	HSIinitialize(0,0,0); //HSI initialization

	if(toLoad!=NULL) {
		if(!NavLoadFlightPlan(toLoad)) logText("ERROR: while opening: %s\n",toLoad);
		else logText("Flight plan loaded\n");
		free(toLoad);
	}

	//Start the GPS using the traditional NMEA parser
	if(!NMEAreaderStartRead()) logText("ERROR: NMEAreader failed to start.\n");
	else logText("NMEAreader started.\n");

	//Start the GPS using SiRFreader //FIXME: WARNING dosen't work
	//if(!SiRFreaderStartRead()) logText("ERROR: SiRFreader failed to start.\n");
	//else logText("SiRF reader started.\n");

	//Start the track recorder
	BlackBoxStart();
	logText("BlackBox recorder started.\n");

	if(toLoad!=NULL) {
		//Here we wait for a touch to start the navigation
		while(!TsScreen_touch(&x,&y));

		//Here we start the navigation
		float timestamp=gps.timestamp;
		if(timestamp==-1) timestamp=getCurrentTime();
		NavStartNavigation(timestamp);
		logText("Navigation started.\n");

		//Here we wait for a touch to reverse the route
		while(!TsScreen_touch(&x,&y));

		//Stop the recorder in order to finalize the track of the first way
		BlackBoxClose();

		//Here we reverse the route
		NavReverseRoute();
		logText("Flight plan reversed.\n");

		//Start the track recorder for the return way
		BlackBoxStart();
		logText("BlackBox recorder restarted.\n");

		timestamp=gps.timestamp;
		if(timestamp==-1) timestamp=getCurrentTime();
		NavStartNavigation(timestamp);
		logText("Navigation re-started.\n");
	}

	/*
	 //and here the SiRF test implemented by tomplayer:
	 //Draw two lines to show that the test is started
	 logText("Inizio test SiRF implemented by tomplayer.\n");//////////
	 DrawTwoPointsLine(10,10,200,200,0xF000);////////////////////////
	 DrawTwoPointsLine(20,10,210,200,0xFF00);////////////////
	 FbRender_Flush();////////////////

	 struct gps_data info;
	 int disp_seq = 0;
	 gps_init();
	 int countdown=10000;
	 while(countdown>0) {
	 if(gps_update()==-1) {
	 gps_get_data(&info);
	 if(info.seq!=disp_seq) {
	 time_t curr_time;
	 struct tm * ptm;
	 disp_seq = info.seq;
	 logText("Lat  : %i°%i'\n",info.lat_deg,info.lat_mins);
	 logText("Long : %i°%i'\n",info.long_deg,info.long_mins);
	 logText("Alt  : %i,%im\n",info.alt_cm/100,info.alt_cm % 100);
	 logText("Sats : %i\n",info.sat_nb);
	 logText("vit  : %ikm/h\n",info.speed_kmh);
	 logText("TimeG: %s",asctime(&info.time));
	 time(&curr_time);
	 ptm=localtime(&curr_time);
	 logText("TimeS: %02d : %02d\n",ptm->tm_hour,ptm->tm_min );
	 }
	 usleep(100000);
	 }
	 countdown--;
	 }
	 gps_close();
	 //Draw two lines to show that the test is finished
	 DrawTwoPointsLine(30,10,220,200,0x0F00);/////////////////////////
	 DrawTwoPointsLine(40,10,230,200,0x00F0);///////////////////////
	 FbRender_Flush();//////////////////
	 logText("Fine test SiRF implemented by tomplayer.\n");//////////
	 */

	//Exit "button"
	FbRender_BlitText(screen.width-(5*8),0,0xffff,0xf000,0,"exit");
	FbRender_Flush();

	short DoExit=0;
	while(!DoExit) {
		while(!TsScreen_touch(&x,&y)); //wait user touches the screen
		if((x>screen.width-(5*8))&&(y<40)) DoExit=1;
	}

	//Clean and Close all the stuff
	NMEAreaderClose();
	logText("GPS reader NMEA terminated.\n");
	//SiRFreaderClose();
	//logText("SiRF reader terminated.\n");
	free(config.GPSdevName);
	NavClose();
	BlackBoxClose();
	logText("BlackBox recorder closed.\n");
	free(config.tomtomModel);
	free(config.serialNumber);
	if(numGPXfiles>0) do { //free the list of GPX files
		currFile=fileList;
		free(currFile->name);
		fileList=currFile->next;
		free(currFile);
	} while(fileList!=NULL);
	logText("Everything terminated, goodbye!\n");
	if(logFile!=NULL) fclose(logFile);
	TsScreen_Exit();
	FbRender_Close();
	pthread_exit(NULL);
	exit(EXIT_SUCCESS);
}

