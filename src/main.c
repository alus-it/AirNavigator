//============================================================================
// Name        : main.c
// Since       : 8/7/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 5/11/2013
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
#include "FbRender.h"
#include "TsScreen.h"
#include "NMEAreader.h"
#include "Navigator.h"
#include "AirCalc.h"
#include "BlackBox.h"
#include "HSI.h"
//#include "SiRFreader.h"
#include "Geoidal.h"

#ifndef VERSION
#define VERSION "0.2.7"
#endif

//TODO: a common base path predefined: /mnt/sdcard/AirNavigator/

typedef struct fileName {
	int seqNo;             //sequence number
	char *name;            //name of the file
	struct fileName *prev; //Pointer to the previous filename in the list
	struct fileName *next; //Pointer to the next filename in the list
} *fileEntry;

pthread_mutex_t logMutex=PTHREAD_MUTEX_INITIALIZER;
FILE *logFile=NULL;

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

void releaseAll() {
	if(logFile!=NULL) fclose(logFile);
	TsScreenClose();
	FbRender_Close();
	pthread_exit(NULL);
}

int main(int argc, char** argv) {
	logFile=fopen("/mnt/sdcard/AirNavigator/log.txt","w"); //create log file
	if(logFile==NULL) {
		printf("ERROR: Unable to create the logFile file!\n");
		releaseAll();
		exit(EXIT_FAILURE);
	}
	if(!FbRender_Open()){
		logText("ERROR: Unable to start the Frame Buffer renderer!\n");
		releaseAll();
		exit(EXIT_FAILURE);
	}
	if(!TsScreenStart()) {
		logText("ERROR: Unable to start the Touch Screen manager!\n");
		releaseAll();
		exit(EXIT_FAILURE);
	}

	FbRender_Flush();
	initConfig(); //initialize the configuration with the default values
	FbRender_Flush();
	logText("AirNavigator v. %s - Compiled: %s %s - www.alus.it\n",VERSION,__DATE__,__TIME__);

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
	fileEntry fileList=NULL, currFile=NULL; //the list of the found GPX flight plans and the pointer to the current one
	int numGPXfiles=0;
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

	condVar_t signal=TsScreenGetCondVar();

	//Display a "menu" with the list of GPX files
	char *toLoad=NULL; //the path to the chosen GPX file to be loaded
	if(numGPXfiles>0) {
		currFile=fileList;
		short DoLoad=0;
		FbRender_BlitText(100,20,colorSchema.dirMarker,colorSchema.background,0,"AirNavigator  v. %s",VERSION);
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
			pthread_mutex_lock(&signal->lastTouchMutex);
			pthread_cond_wait(&signal->lastTouchSignal,&signal->lastTouchMutex); //wait user presses a "button"
			TS_EVENT lt=TsScreenGetLastTouch();
			if(lt.y>200) {
				if(lt.x<80 && currFile->prev!=NULL) currFile=currFile->prev;
				else if(lt.x>200 && lt.x<300) {
					asprintf(&toLoad,"%s%s","/mnt/sdcard/AirNavigator/Routes/",currFile->name);
					DoLoad=1;
				} else if(lt.x>350 && currFile->next!=NULL) currFile=currFile->next;
			}
			pthread_mutex_unlock(&signal->lastTouchMutex);
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
		pthread_mutex_lock(&signal->lastTouchMutex);
		pthread_cond_wait(&signal->lastTouchSignal,&signal->lastTouchMutex);
		pthread_mutex_unlock(&signal->lastTouchMutex);

		//Here we start the navigation
		float timestamp=gps.timestamp;
		if(timestamp==-1) timestamp=getCurrentTime();
		NavStartNavigation(timestamp);
		logText("Navigation started.\n");

		//Here we wait for a touch to reverse the route
		pthread_mutex_lock(&signal->lastTouchMutex);
		pthread_cond_wait(&signal->lastTouchSignal,&signal->lastTouchMutex);
		pthread_mutex_unlock(&signal->lastTouchMutex);

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

	//Exit "button"
	FbRender_BlitText(screen.width-(5*8),0,0xffff,0xf000,0,"exit");
	FbRender_Flush();

	short DoExit=0;
	while(!DoExit) {
		pthread_mutex_lock(&signal->lastTouchMutex);
		pthread_cond_wait(&signal->lastTouchSignal,&signal->lastTouchMutex); //wait user touches the screen
		TS_EVENT lt=TsScreenGetLastTouch();
		if((lt.x>screen.width-(5*8))&&(lt.y<40)) {
			DoExit=1;
			logText("Touch to exit detected\n");
		}
		pthread_mutex_unlock(&signal->lastTouchMutex);
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
	logText("Releasing all... Goodbye!\n");
	releaseAll();
	exit(EXIT_SUCCESS);
}

