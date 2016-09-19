//============================================================================
// Name        : main.c
// Since       : 8/7/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2016 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 19/9/2016
// Description : main function of the AirNavigator program for TomTom devices
//============================================================================

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
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

#define VERSION "0.3.2"

typedef struct fileName {
	int seqNo;             //sequence number
	char *name;            //name of the file
	struct fileName *prev; //Pointer to the previous filename in the list
	struct fileName *next; //Pointer to the next filename in the list
} *fileEntry;

struct mainStruct {
	enum mainStatus status;           //Main status, it is the screen shown: main menu, HSI or select GPX file
	char *bottomBarMsg;               //Text confirmation message shown at the bottom of main menu
	unsigned short bottomBarMsgColor; //Color of text confirmation message
};

void releaseAll(void);
int showMessage(unsigned short color, bool logMessage, const char *args, ...);

static struct mainStruct mainData = {
	.status=MAIN_NOT_INIT,
	.bottomBarMsg=NULL
};

int main(int argc, char** argv) {
	if(openLog()) //if the log file has been created...
		if(FBrenderOpen()) //if the frame buffer render is started
			if(TSreaderStart()) mainData.status=MAIN_DISPLAY_MENU; //if the touch screen listening thread is started
			else printLog("ERROR: Unable to start the Touch Screen manager!\n");
		else printLog("ERROR: Unable to start the Frame Buffer renderer!\n");
	else printf("ERROR: Unable to create the logFile file!\n");
	if(mainData.status!=MAIN_DISPLAY_MENU) {
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
	struct dirent *entry=NULL;
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
		if(numGPXfiles==0) showMessage(config.colorSchema.warning,true,"WARNING: No GPX flight plan files found.");
		else {
			showMessage(config.colorSchema.ok,true,"Found %d GPX flight plan files.",numGPXfiles);
			currFile=fileList;
		}
	} else showMessage(config.colorSchema.caution,true,"ERROR: could not open the Routes directory.");
	if(!GPSreceiverStart()) showMessage(config.colorSchema.caution,true,"ERROR: GPSreceiver failed to start."); //Start GPSrecveiver
	//TODO: if GPS failed to start many buttons should be disabled...
	bool doExit=false;
	TS_EVENT lastTouch; //data of the last event from the touch screen
	char *toLoad=NULL; //the path to the chosen GPX file to be loaded
	int numWPloaded=0; //the number of waypoints loaded from the selected flight plan
	while(!doExit) { //Main loop
		FBrenderClear(0,screen.height,config.colorSchema.background);
		switch(mainData.status) { //Depending on status display the proper screen
			case MAIN_DISPLAY_MENU:
				FBrenderBlitText(10,10,config.colorSchema.dirMarker,config.colorSchema.background,false,"AirNavigator v.%s",VERSION);
				FBrenderBlitText(200,10,config.colorSchema.magneticDir,config.colorSchema.background,true,"http://www.alus.it/airnavigator");
				DrawButton(20,50,numGPXfiles>0,"Load flight plan");
				DrawButton(20,90,NavGetStatus()==NAV_STATUS_TO_START_NAV,"Start navigation");
				DrawButton(20,130,numWPloaded>1,"Reverse flight plan");
				DrawButton(20,170,numWPloaded>0,"Unload flight plan");
				DrawButton(220,50,true,"Show HSI");
				DrawButton(220,90,true,BlackBoxIsStarted()?"Stop Track Recorder":"Start Track Recorder");
				DrawButton(220,130,BlackBoxIsStarted(),BlackBoxIsPaused()?"Resume Track Recorder":"Pause Track Recorder");
				DrawButton(220,210,true,"EXIT");
				if(mainData.bottomBarMsg!=NULL) FBrenderBlitText(10,260,mainData.bottomBarMsgColor,config.colorSchema.background,false,"%s                                                         ",mainData.bottomBarMsg); //render confirmation msg
				break;
			case MAIN_DISPLAY_SELECT_ROUTE: //Display the select GPX flight plan screen
				FBrenderBlitText(20,20,config.colorSchema.dirMarker,config.colorSchema.background,0,"Select and load the desired GPX flight plan");
				FBrenderBlitText(20,35,config.colorSchema.text,config.colorSchema.background,0,"%d GPX flight plans found.",numGPXfiles);
				FBrenderBlitText(20,60,config.colorSchema.text,config.colorSchema.background,0,"Selected GPX flight plan:");
				FBrenderBlitText(20,70,config.colorSchema.warning,config.colorSchema.background,0,"%s                                         ",currFile->name); //print the name of the current file
				DrawButton(20,90,currFile->prev!=NULL,"<< Previous");
				DrawButton(220,90,currFile->next!=NULL,"    Next >>");
				DrawButton(220,210,currFile!=NULL,"    LOAD");
				DrawButton(20,210,true,"Back to menu");
			break;
			case MAIN_DISPLAY_HSI: //Display HSI
				NavRedrawNavInfo();
				break;
			case MAIN_DISPLAY_SUNRISE_SUNSET: // Display ephemerides
				NavRedrawEphemeridalInfo();
				break;
			default:
				break;
		} //end of display switch
		FBrenderFlush();
		TSreaderGetTouch(&lastTouch); //wait that the user touches the screen and get the coordinates of the touch
		switch(mainData.status) { //depending on which screen we are process the input touch
			case MAIN_DISPLAY_MENU: //here process main menu input
				if(lastTouch.x>=20 && lastTouch.x<=200) { //touched the first column of buttons
					if(lastTouch.y>=50 && lastTouch.y<=80 && numGPXfiles>0) mainData.status=MAIN_DISPLAY_SELECT_ROUTE; //touched load route button
					if(lastTouch.y>=90 && lastTouch.y<=120 && NavGetStatus()==NAV_STATUS_TO_START_NAV) { //touched start navigation button
						NavStartNavigation();
						mainData.status=MAIN_DISPLAY_HSI;
					}
					if(lastTouch.y>=130 && lastTouch.y<=160 && numWPloaded>1) { //touched reverse route button
						if(NavReverseRoute()) showMessage(config.colorSchema.ok,false,"Route reversed."); //reverse the route
						else showMessage(config.colorSchema.caution,true,"ERROR: Failed to reverse route.");
					}
					if(lastTouch.y>=170 && lastTouch.y<=200 && numWPloaded>0) { //touched unload route button
						NavClearRoute();
						numWPloaded=0;
						currFile=fileList;
						showMessage(config.colorSchema.ok,false,"Route unloaded.");
					}
				} else if(lastTouch.x>=220 && lastTouch.x<=400) { //touched second column of buttons
					if(lastTouch.y>=50 && lastTouch.y<=80) mainData.status=MAIN_DISPLAY_HSI; //touched show HSI button
					if(lastTouch.y>=90 && lastTouch.y<=120) { //touched start stop track recorder button
						if(BlackBoxIsStarted()) {
							BlackBoxClose();
							showMessage(config.colorSchema.ok,true,"Track recorder stopped.");
						} else {
							BlackBoxStart();
							showMessage(config.colorSchema.ok,true,"Track recorder started.");
						}
					}
					if(lastTouch.y>=130 && lastTouch.y<=160 && BlackBoxIsStarted()) { //touched pause resume track recorder button
						if(BlackBoxIsPaused()) {
							BlackBoxResume();
							showMessage(config.colorSchema.ok,false,"Track recorder resumed.");
						} else {
							BlackBoxPause();
							showMessage(config.colorSchema.ok,false,"Track recorder paused.");
						}
					}
					if(lastTouch.y>=210 && lastTouch.y<=240) { //touched exit button
						doExit=true;
						showMessage(config.colorSchema.warning,false,"Exit: releasing all... Goodbye!"); //Show goodbye message
						FBrenderBlitText(10,260,mainData.bottomBarMsgColor,config.colorSchema.background,false,"%s                                                         ",mainData.bottomBarMsg);
						FBrenderFlush();
					}
				}
				break;
			case MAIN_DISPLAY_SELECT_ROUTE: //here process user input in select route screen
				if(lastTouch.y>=90 && lastTouch.y<=120) { //user touched at the height of prev and next buttons
					if(lastTouch.x>=20 && lastTouch.x<=200 && currFile->prev!=NULL) currFile=currFile->prev;  //user touched prev button
					else if(lastTouch.x>=220 && lastTouch.x<=400 && currFile->next!=NULL) currFile=currFile->next; //user touched next button
				} else if(lastTouch.y>=210 && lastTouch.y<=240) { //user touched at the height of back, load buttons
					if(lastTouch.x>=20 && lastTouch.x<=200) {  //user touched back button
						mainData.status=MAIN_DISPLAY_MENU; //go back to main menu
						free(mainData.bottomBarMsg); //remove previous message
						mainData.bottomBarMsg=NULL;
					} else if(lastTouch.x>=220 && lastTouch.x<=400) { //user touched LOAD button
						asprintf(&toLoad,"%s%s%s",BASE_PATH,"Routes/",currFile->name);
						numWPloaded=NavLoadFlightPlan(toLoad); //Attempt to load the flight plan
						if(numWPloaded<1) { //if load route failed
							if(toLoad!=NULL) showMessage(config.colorSchema.caution,true,"ERROR: while opening: %s",toLoad);
							else showMessage(config.colorSchema.caution,true,"ERROR: NULL pointer to the route file to be loaded.");
						} else showMessage(config.colorSchema.ok,true,"Loaded route: %s - %d WayPoints",currFile->name,numWPloaded);
						free(toLoad);
						toLoad=NULL;
						mainData.status=MAIN_DISPLAY_MENU;
					}
				}
				break;
			case MAIN_DISPLAY_HSI: //here process the user input the HSI screen
			case MAIN_DISPLAY_SUNRISE_SUNSET: // and in the ephemeides screen
				mainData.status=MAIN_DISPLAY_MENU; //a touch anywhere here brings back to main menu
				if(numWPloaded>0) showMessage(config.colorSchema.ok,false,"Loaded route: %s - %d WayPoints",currFile->name,numWPloaded);
				else { //Nothing to display
					free(mainData.bottomBarMsg);
					mainData.bottomBarMsg=NULL;
				}
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
	free(mainData.bottomBarMsg);
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
	return mainData.status;
}

int showMessage(unsigned short color, bool logMessage, const char *args, ...) {
	int done=-1;
	if(args!=NULL) {
		if(mainData.bottomBarMsg!=NULL) {
			free(mainData.bottomBarMsg);
			mainData.bottomBarMsg=NULL;
		}
		va_list arg;
		va_start(arg,args);
		done=vasprintf(&mainData.bottomBarMsg,args,arg);
		if(done) {
			mainData.bottomBarMsgColor=color;
			if(logMessage) printLog("%s\n",mainData.bottomBarMsg);
		}
	}
	return done;
}
