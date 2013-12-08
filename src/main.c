//============================================================================
// Name        : main.c
// Since       : 8/7/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 8/12/2013
// Description : main function of the AirNavigator program for TomTom devices
//============================================================================

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <pthread.h>
#include "Common.h"
#include "Configuration.h"
#include "FBrender.h"
#include "TSreader.h"
#include "GPSreceiver.h"
#include "Navigator.h"
#include "AirCalc.h"
#include "BlackBox.h"
#include "HSI.h"

#ifndef VERSION
#define VERSION "0.3.0"
#endif


typedef struct fileName {
	int seqNo;             //sequence number
	char *name;            //name of the file
	struct fileName *prev; //Pointer to the previous filename in the list
	struct fileName *next; //Pointer to the next filename in the list
} *fileEntry;

void releaseAll(void);

static enum mainStatus status=MAIN_NOT_INIT;

int main(int argc, char** argv) {
	if(openLog()) //if the log file has been created...
		if(FBrenderOpen()) //if the frame buffer render is started
			if(TSreaderStart()) status=MAIN_DISPLAY_MENU; //if the touch screen listening thread is started
			else printLog("ERROR: Unable to start the Touch Screen manager!\n");
		else printLog("ERROR: Unable to start the Frame Buffer renderer!\n");
	else printf("ERROR: Unable to create the logFile file!\n");
	if(status!=MAIN_DISPLAY_MENU) {
		releaseAll();
		exit(EXIT_FAILURE);
	}
	FBrenderFlush();
	printLog("Started AirNavigator v. %s - Compiled: %s %s - www.alus.it\n",VERSION,__DATE__,__TIME__);
	bool allOK=true;
	FILE *fd=NULL;
	fd=fopen("/proc/barcelona/modelname","r"); //Read TomTom model
	if(fd!=NULL) {
		char buf[30];
		char *ret=fgets(buf,29,fd);
		if(ret==buf) {
			if(buf[strlen(buf)-1]=='\n') buf[strlen(buf)-1]='\0';
			config.tomtomModel=strdup(buf);
			printLog("TomTom model name: %s\n",config.tomtomModel);
		} else allOK=false;
		fclose(fd);
	} else allOK=false;
	if(!allOK) {
		printLog("ERROR: unable to read TomTom device model name.\n");
		config.tomtomModel=strdup("UNKNOWN");
		allOK=true;
	}
	fd=NULL;
	fd=fopen("/mnt/flash/sysfile/id","r"); //Read device serial number
	if(fd!=NULL) {
		char buf[30];
		char *ret=fgets(buf,29,fd);
		if(ret==buf) {
			config.serialNumber=strdup(buf);
			printLog("TomTom device serial number ID: %s\n",config.serialNumber);
		} else allOK=false;
		fclose(fd);
	} else allOK=false;
	if(!allOK) {
		printLog("ERROR: unable to read TomTom device serial number ID.\n");
		config.serialNumber=strdup("UNKNOWN");
	}
	printLog("Screen resolution: %dx%d pixel\n",screen.width,screen.height); //logFile screen resolution
	loadConfig(); //Load configuration
	fileEntry fileList=NULL, currFile=NULL; //the list of the found GPX flight plans and the pointer to the current one
	int numGPXfiles=0;
	struct dirent *entry;
	char *routesPath; //... prepare the list of available flight plans found in the routes folder
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
		if(numGPXfiles==0) printLog("WARNING: No GPX flight plans found.\n");
		else {
			printLog("Created list of %d GPX files.\n",numGPXfiles);
			currFile=fileList;
		}
	} else printLog("ERROR: could not open the Routes directory.\n");

	if(!GPSreceiverStart()) printLog("ERROR: GPSreceiver failed to start.\n"); //Start GPSrecveiver
	//TODO: if GPS failed to start many buttons should be disabled...

	//TODO: put all those possible error messages in a string to be shown in the confirmation bar in the menu

	bool doExit=false;
	TS_EVENT lastTouch; //data of the last event from the touch screen
	char *toLoad=NULL; //the path to the chosen GPX file to be loaded
	int numWPloaded=0; //the number of waypoints loaded from the selected flight plan
	while(!doExit) { //Main loop
		switch(status) { //Depending on status display the proper screen
			case MAIN_DISPLAY_MENU:
				FBrenderClear(0,screen.height,config.colorSchema.background);
				FBrenderBlitText(10,10,config.colorSchema.dirMarker,config.colorSchema.background,false,"AirNavigator v.%s",VERSION);
				FBrenderBlitText(200,10,config.colorSchema.magneticDir,config.colorSchema.background,true,"http://www.alus.it/airnavigator");
				//TODO: show the UTC time and fix in the menu???
				DrawButton(20,50,numGPXfiles>0,"Load flight plan");
				DrawButton(20,90,NavGetStatus()==NAV_STATUS_TO_START_NAV,"Start navigation");
				DrawButton(20,130,numWPloaded>1,"Reverse flight plan");
				DrawButton(20,170,numWPloaded>0,"Unload flight plan");
				DrawButton(220,50,true,"Show HSI");
				DrawButton(220,90,true,BlackBoxIsStarted()?"Stop Track Recorder":"Start Track Recorder");
				DrawButton(220,130,BlackBoxIsStarted(),BlackBoxIsPaused()?"Resume Track Recorder":"Pause Track Recorder");
				DrawButton(220,210,true,"EXIT");
				//TODO: make a bottom bar with advice messages like: FP loaded with n WP ecc...
				FBrenderFlush();
				break;
			case MAIN_DISPLAY_SELECT_ROUTE: //Display the select GPX flight plan screen
				FBrenderClear(0,screen.height,config.colorSchema.background); //draw the main screen
				FBrenderBlitText(20,20,config.colorSchema.dirMarker,config.colorSchema.background,0,"Select and load the desired GPX flight plan");
				FBrenderBlitText(20,35,config.colorSchema.text,config.colorSchema.background,0,"%d GPX flight plans found.",numGPXfiles);
				FBrenderBlitText(20,60,config.colorSchema.text,config.colorSchema.background,0,"Selected GPX flight plan:");
				FBrenderBlitText(20,70,config.colorSchema.warning,config.colorSchema.background,0,"%s                                         ",currFile->name); //print the name of the current file
				DrawButton(20,90,currFile->prev!=NULL,"<< Previous");
				DrawButton(220,90,currFile->next!=NULL,"    Next >>");
				DrawButton(220,210,currFile->next!=NULL,"    LOAD");
				FBrenderFlush();
			break;
			case MAIN_DISPLAY_HSI: //Display HSI, start reading from GPS and the track recorder
				FBrenderClear(0,screen.height,config.colorSchema.background); //draw the main screen
				HSIinitialize(0,0,0); //HSI initialization
				//TODO here is necessary to draw all the instrumentation...
				break;
			default:
				break;
		} //end of display switch
		TSreaderGetTouch(&lastTouch); //wait that the user touches the screen and get the coordinates of the touch
		switch(status) { //depending on the status process the input touch
			case MAIN_DISPLAY_MENU: //here process main menu input
				if(lastTouch.x>=20 && lastTouch.x<=200) { //touched the first column of buttons
					if(lastTouch.y>=50 && lastTouch.y<=80 && numGPXfiles>0) status=MAIN_DISPLAY_SELECT_ROUTE; //touched load route button
					if(lastTouch.y>=90 && lastTouch.y<=120 && NavGetStatus()==NAV_STATUS_TO_START_NAV) { //touched start navigation button
						status=MAIN_DISPLAY_HSI;
						NavStartNavigation();
					}
					if(lastTouch.y>=130 && lastTouch.y<=160 && numWPloaded>1) NavReverseRoute();//touched reverse route button
					if(lastTouch.y>=170 && lastTouch.y<=200 && numWPloaded>0) { //touched unload route button
						NavClearRoute();
						numWPloaded=0;
					}
				} else if(lastTouch.x>=220 && lastTouch.x<=400) { //touched second column of buttons
					if(lastTouch.y>=50 && lastTouch.y<=80) status=MAIN_DISPLAY_HSI; //touched show HSI button
					if(lastTouch.y>=90 && lastTouch.y<=120) { //touched start stop track recorder button
						if(BlackBoxIsStarted()) BlackBoxClose();
						else BlackBoxStart();
					}
					if(lastTouch.y>=130 && lastTouch.y<=160 && BlackBoxIsStarted()) { //touched pause resume track recorder button
						if(BlackBoxIsPaused()) BlackBoxResume();
						else BlackBoxPause();
					}
					if(lastTouch.y>=210 && lastTouch.y<=240) doExit=true; //touched exit button
				}
				break;
			case MAIN_DISPLAY_SELECT_ROUTE: //here process user input in select route screen
				if(lastTouch.y>=90 && lastTouch.y<=120) { //user touched at the height of prev and next buttons
					if(lastTouch.x>=20 && lastTouch.x<=200 && currFile->prev!=NULL) { //user touched prev button
						currFile=currFile->prev;
					} else if(lastTouch.x>=220 && lastTouch.x<=400 && currFile->next!=NULL) { //user touched next button
						currFile=currFile->next;
					}
				} else if(lastTouch.y>=210 && lastTouch.y<=240 && lastTouch.x>=220 && lastTouch.x<=400) { //user touched LOAD button
					asprintf(&toLoad,"%s%s%s",BASE_PATH,"Routes/",currFile->name);
					numWPloaded=NavLoadFlightPlan(toLoad); //Attempt to load the flight plan
					if(numWPloaded<1) { //if load route failed
						if(toLoad!=NULL) printLog("ERROR: while opening: %s\n",toLoad);
						else printLog("ERROR: NULL pointer to the route file to be loaded.\n");
					} else printLog("Loaded route with %d WayPoints.\n\n",numWPloaded);
					free(toLoad);
					status=MAIN_DISPLAY_MENU;
				}
				break;
			case MAIN_DISPLAY_HSI: //here process the user input the HSI screen
				status=MAIN_DISPLAY_MENU; //a touch anywhere in the HSI screen bring back to main menu
				break;
			default:
				doExit=true;
				break;
		} //end of user input processing switch
	} //end of main loop
	GPSreceiverClose(); //Clean and Close all ...
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
	releaseAll();
	exit(EXIT_SUCCESS);
}

void releaseAll(void) {
	closeLog();
	TSreaderClose();
	FBrenderClose();
	pthread_exit(NULL);
}

enum mainStatus getMainStatus(void) {
	return status;
}
