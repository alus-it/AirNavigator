//============================================================================
// Name        : main.c
// Since       : 8/7/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 16/11/2013
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
#include "FBrender.h"
#include "TSreader.h"
#include "NMEAreader.h"
#include "Navigator.h"
#include "AirCalc.h"
#include "BlackBox.h"
#include "HSI.h"
//#include "SiRFreader.h"
#include "Geoidal.h"

#ifndef VERSION
#define VERSION "0.2.9"
#endif

#define MAIN_STATUS_NOT_INIT    -1
#define MAIN_STATUS_INITIALIZED  0
#define MAIN_STATUS_START_GPS    1
#define MAIN_STATUS_NOT_LOADED   2
#define MAIN_STATUS_SELECT_ROUTE 3
#define MAIN_STATUS_READY_TO_NAV 4
#define MAIN_STATUS_FLY_ROUTE    5
#define MAIN_STATUS_FLY_REVERSED 6
#define MAIN_STATUS_WAIT_EXIT    7

typedef struct fileName {
	int seqNo;             //sequence number
	char *name;            //name of the file
	struct fileName *prev; //Pointer to the previous filename in the list
	struct fileName *next; //Pointer to the next filename in the list
} *fileEntry;

void releaseAll(void);

pthread_mutex_t logMutex=PTHREAD_MUTEX_INITIALIZER;
FILE *logFile=NULL;

int main(int argc, char** argv) {
	int status=MAIN_STATUS_NOT_INIT;
	char *logPath;
	asprintf(&logPath,"%slog.txt",BASE_PATH);
	logFile=fopen(logPath,"w"); //create log file
	free(logPath);
	if(logFile!=NULL) //if the log file has been created...
		if(FBrenderOpen()) //if the frame buffer render is started
			if(TSreaderStart()) { //if the touch screen listening thread is started
				initConfig(); //initialize the configuration with the default values
				status=MAIN_STATUS_INITIALIZED;
			} else logText("ERROR: Unable to start the Touch Screen manager!\n");
		else logText("ERROR: Unable to start the Frame Buffer renderer!\n");
	else printf("ERROR: Unable to create the logFile file!\n");
	if(status!=MAIN_STATUS_INITIALIZED) {
		releaseAll();
		exit(EXIT_FAILURE);
	}
	FBrenderFlush();
	logText("AirNavigator v. %s - Compiled: %s %s - www.alus.it\n",VERSION,__DATE__,__TIME__);

	FILE *fd=NULL;
	fd=fopen("/proc/barcelona/modelname","r"); //Read TomTom model
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

	fd=NULL;
	fd=fopen("/mnt/flash/sysfile/id","r"); //Read device serial number
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
	//FBrenderFlush();

	loadConfig(); //Load configuration

	//Prepare the list of available flight plans found in the routes folder
	fileEntry fileList=NULL, currFile=NULL; //the list of the found GPX flight plans and the pointer to the current one
	int numGPXfiles=0;
	struct dirent *entry;
	char *routesPath;
	asprintf(&routesPath,"%sRoutes",BASE_PATH);
	DIR *dir = opendir(routesPath);
	free(routesPath);
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
		else logText("List of GPX files created.\n");
	} else logText("ERROR: could not open the Routes directory.\n");

	if(numGPXfiles>0) { //Start to prepare the "menu" to select the route
		status=MAIN_STATUS_SELECT_ROUTE;
		currFile=fileList;
		FBrenderBlitText(100,20,colorSchema.dirMarker,colorSchema.background,0,"AirNavigator  v. %s",VERSION);
		FBrenderBlitText(100,30,colorSchema.magneticDir,colorSchema.background,0,"http://www.alus.it/airnavigator");
		FBrenderBlitText(20,60,colorSchema.text,colorSchema.background,0,"Select a GPX flight plan:");
		FBrenderBlitText(200,240,colorSchema.text,colorSchema.background,0,"LOAD");
	} else status=MAIN_STATUS_START_GPS;

	short doExit=0;
	condVar_t signal=TSreaderGetCondVar();
	TS_EVENT lastTouch; //data of the last event from the touch screen
	short waitTouch=0; //flag to know if we are waining for user input on the touch screen
	char *toLoad=NULL; //the path to the chosen GPX file to be loaded
	int numWPloaded=0; //the number of waypoints loaded from the selected flight plan
	while(!doExit) { //Main loop
		if(waitTouch) { //if needed wait for user input
			pthread_mutex_lock(&signal->lastTouchMutex);
			pthread_cond_wait(&signal->lastTouchSignal,&signal->lastTouchMutex); //wait user touches the screen
			lastTouch=TSreaderGetLastTouch(); //get the coordinates of the touch
			pthread_mutex_unlock(&signal->lastTouchMutex);
		}
		switch(status) { //Main status machine
			case MAIN_STATUS_SELECT_ROUTE: { //Display a "menu" with the list of GPX files
				if(numGPXfiles>0) {
					FBrenderBlitText(20,70,colorSchema.cdi,colorSchema.background,0,"%s                                         ",currFile->name); //print the name of the current file
					if(currFile->prev!=NULL) FBrenderBlitText(20,240,colorSchema.text,colorSchema.background,0,"<< Prev");
					else FBrenderBlitText(20,240,colorSchema.text,colorSchema.background,0,"       ");
					if(currFile->next!=NULL) FBrenderBlitText(350,240,colorSchema.text,colorSchema.background,0,"Next >>");
					else FBrenderBlitText(350,240,colorSchema.text,colorSchema.background,0,"       ");
					FBrenderFlush();
					if(waitTouch) {
						if(lastTouch.y>200) { //user touched lower part of the screen
							if(lastTouch.x<80 && currFile->prev!=NULL) { //user touched prev button
								currFile=currFile->prev;
								waitTouch=0;
							} else if(lastTouch.x>200 && lastTouch.x<300) { //user touched LOAD button
								asprintf(&toLoad,"%s%s%s",BASE_PATH,"Routes/",currFile->name);
								status=MAIN_STATUS_START_GPS;
								waitTouch=0;
							} else if(lastTouch.x>350 && currFile->next!=NULL) { //user touched next button
								currFile=currFile->next;
								waitTouch=0;
							}
						}
					} else waitTouch=1;
				}
			} break;
			case MAIN_STATUS_START_GPS: //Start reading from GPS and the track recorder
				FBrenderClear(0,screen.height,colorSchema.background); //draw the main screen
				HSIinitialize(0,0,0); //HSI initialization
				if(!NMEAreaderStartRead()) logText("ERROR: NMEAreader failed to start.\n"); //Start the GPS using the traditional NMEA parser
				//if(!SiRFreaderStartRead()) logText("ERROR: SiRFreader failed to start.\n"); //... or start the GPS using the newer SiRF parser (not working yet)
				BlackBoxStart(); //Start the track recorder
				if(toLoad!=NULL) status=MAIN_STATUS_READY_TO_NAV;
				else status=MAIN_STATUS_NOT_LOADED;
				break;
			case MAIN_STATUS_NOT_LOADED: //No route loaded, but display anyway data from GPS
				FBrenderBlitText(screen.width-(5*8),0,0xffff,0xf000,0,"exit"); //Exit "button"
				FBrenderFlush();
				waitTouch=1; //Wait for a touch on the exit button
				status=MAIN_STATUS_WAIT_EXIT;
				break;
			case MAIN_STATUS_READY_TO_NAV: //A route is available to be flown
				numWPloaded=NavLoadFlightPlan(toLoad); //Attempt to load the flight plan
				if(numWPloaded<1) { //if load route failed
					if(toLoad!=NULL) {
						logText("ERROR: while opening: %s\n",toLoad);
						free(toLoad);
					} else logText("ERROR: NULL pointer to the route file to be loaded.\n");
					status=MAIN_STATUS_NOT_LOADED;
					waitTouch=0;
					break;
				} else logText("Loaded route with %d WayPoints.\n\n",numWPloaded);
				free(toLoad);
				waitTouch=1; //Here we wait for a touch to start the navigation
				status=MAIN_STATUS_FLY_ROUTE;
				break;
			case MAIN_STATUS_FLY_ROUTE: { //Navigation can start
				float timestamp=gps.timestamp; //Get actual time from GPS
				if(timestamp==-1) timestamp=getCurrentTime(); //if not valid get it from internal clock
				NavStartNavigation(timestamp); //Start the navigation
				waitTouch=1; //Wait for a touch to reverse the route or to exit
				if(numWPloaded>1) status=MAIN_STATUS_FLY_REVERSED; //if there is more than 1 WP reverse the route
				else {  //otherwise...
					FBrenderBlitText(screen.width-(5*8),0,0xffff,0xf000,0,"exit"); //... show exit button ...
					status=MAIN_STATUS_WAIT_EXIT; // ...and wait that user presses it
				}
			} break;
			case MAIN_STATUS_FLY_REVERSED: { //Flight plan can be reversed
				BlackBoxClose(); //Stop the recorder in order to finalize the track of the first way
				NavReverseRoute(); //reverse the route
				BlackBoxStart(); //Restart the track recorder for the return way
				float timestamp=gps.timestamp;
				if(timestamp==-1) timestamp=getCurrentTime();
				NavStartNavigation(timestamp);
				FBrenderBlitText(screen.width-(5*8),0,0xffff,0xf000,0,"exit"); //Show exit "button"
				FBrenderFlush();
				waitTouch=1; //Here we wait for a touch on the exit button
				status=MAIN_STATUS_WAIT_EXIT;
			} break;
			case MAIN_STATUS_WAIT_EXIT:
				if((lastTouch.x>screen.width-(5*8))&&(lastTouch.y<40)) doExit=1;
				waitTouch=1; //Here we wait for a touch on the exit button
				break;
			default:
				break;
		}
	}

	//Clean and Close all the stuff
	NMEAreaderClose();
	//SiRFreaderClose();
	free(config.GPSdevName);
	NavClose();
	BlackBoxClose();
	free(config.tomtomModel);
	free(config.serialNumber);
	if(numGPXfiles>0) do { //free the list of GPX files
		currFile=fileList;
		free(currFile->name);
		fileList=currFile->next;
		free(currFile);
	} while(fileList!=NULL);
	logText("Releasing all... Goodbye!\n");
	releaseAll();
	exit(EXIT_SUCCESS);
}

void releaseAll(void) {
	if(logFile!=NULL) fclose(logFile);
	TSreaderClose();
	FBrenderClose();
	pthread_exit(NULL);
}

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

