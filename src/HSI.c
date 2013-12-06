//============================================================================
// Name        : HSI.c
// Since       : 16/9/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 5/12/2011
// Description : Draws and updates the Horizontal Situation Indicator
//============================================================================

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "HSI.h"
#include "FBrender.h"
#include "AirCalc.h"
#include "Configuration.h"


struct HSIstruct {
	char *label[12];
	int labelHalfWidth[12];
	int labelHalfHeight;
	int cx,cy; //center
	int mark_start;
	int re; //external radius
	int major_mark;
	int label_pos;
	int dir_display_pos;
	int ri; //internal radius
	int cir; //clear internal radius
	int minor_mark;
	int arrow_end;
	int arrow_side;
	int cdi_border;
	int cdi_end;
	int cdi_pixel_scale;
	int cdi_pixel_tick;
	int cdi_scale_mark;
	int cdi_scale;
	int previousDir;
	int HalfAltScale;
	int PxAltScale;
	double actualDir,actualMagDir,actualCourse,actualCDI;
	long currentAltFt,expectedAltFt;
	int symFuselageL;
	int symFuselageR;
	int symFuselageU;
	int symFuselageD;
	int symWingL;
	int symWingR;
	int symWingU;
	int symWingC;
	int symWingD;
	int symTailL;
	int symTailR;
	int symTailU;
	int symTailC;
	int symTailD;
};

int HSIround(double d);
void rotatePoint(int mx, int my, int *px, int *py, double angle);
void drawCompass(int dir);
void drawLabels(int dir);
void drawCDI(double direction, double course, double cdi);
void diplayCDIvalue(double cdiMt);

static struct HSIstruct HSI = {
	.label={"N","03","06","E","12","15","S","21","24","W","30","33"},
	.labelHalfWidth={2,6,6,2,6,6,2,6,6,2,6,6},
	.labelHalfHeight=4
};

void HSIinitialize(double directionDeg, double courseDeg, double courseDeviationMt) {
	HSI.cx=screen.height/2; //aligned on the left
	HSI.cy=screen.height/2; //half height
	HSI.mark_start=10;
	HSI.re=HSI.cy-HSI.mark_start+1; //external radius
	HSI.major_mark=HSI.mark_start+20;
	HSI.label_pos=HSI.major_mark+10;
	HSI.dir_display_pos=HSI.label_pos+15;
	HSI.ri=HSI.cy-HSI.mark_start-HSI.major_mark; //internal radius
	HSI.cir=HSI.ri+11; //clear internal radius used to clear the CDI but not the external compass
	HSI.minor_mark=HSI.mark_start+12;
	HSI.arrow_end=HSI.major_mark+18;
	HSI.arrow_side=8;
	HSI.cdi_border=HSI.major_mark+31; //ri-ri*sen(45°)=31
	HSI.cdi_end=screen.height-HSI.cdi_border;
	HSI.cdi_pixel_scale=75; //ri*sen(45°)=75
	HSI.cdi_pixel_tick=HSI.cdi_pixel_scale/5;
	HSI.cdi_scale_mark=15;
	HSI.cdi_scale=500; // 0.3NM = 0.56 Km
	HSI.previousDir=-456;
	HSI.actualMagDir=0;
	HSI.actualCourse=-456;
	HSI.actualCDI=0;
	HSI.currentAltFt=-5000;
	HSI.expectedAltFt=-5000;
	HSI.HalfAltScale=500;
	HSI.symFuselageL=HSI.cx-1;
	HSI.symFuselageR=HSI.cx+1;
	HSI.symFuselageU=HSI.cy-18;
	HSI.symFuselageD=HSI.cy+18;
	HSI.symWingL=HSI.cx-16;
	HSI.symWingR=HSI.cx+16;
	HSI.symWingU=HSI.cy-8;
	HSI.symWingC=HSI.cy-7;
	HSI.symWingD=HSI.cy-6;
	HSI.symTailL=HSI.cx-6;
	HSI.symTailR=HSI.cx+6;
	HSI.symTailU=HSI.cy+10;
	HSI.symTailC=HSI.cy+11;
	HSI.symTailD=HSI.cy+12;
	if(screen.height==240) HSI.HalfAltScale=438;
	HSI.PxAltScale=screen.height-12;
	DrawTwoPointsLine(HSI.cx-1,0,HSI.cx-1,HSI.mark_start-4,config.colorSchema.dirMarker);
	DrawTwoPointsLine(HSI.cx,0,HSI.cx,HSI.mark_start,config.colorSchema.dirMarker);
	DrawTwoPointsLine(HSI.cx+1,0,HSI.cx+1,HSI.mark_start-4,config.colorSchema.dirMarker);
	HSIdraw(directionDeg,courseDeg,courseDeviationMt);
	DrawTwoPointsLine(screen.height,6,screen.height,screen.height-6,config.colorSchema.altScale); //the line of the altitude scale
}

int HSIround(double d) {
  int n =(int)d;
  return (d-(double)n)>0.5 ? (int)(n+1) : (int)n;
}

void rotatePoint(int mx, int my, int *px, int *py, double angle) {
  double cos_theta = cos(angle); //convert the angle to a useful form
  double sin_theta = sin(angle);
  int x2 = round((*px-mx)*(cos_theta)-(*py-my)*(sin_theta)); // calc the transformation
  int y2 = round((*px-mx)*(sin_theta)+(*py-my)*(cos_theta));
  *px=x2+mx;
  *py=y2+my;
}

void drawCompass(int dir) { //works with direction as integer
	HSI.previousDir=dir;
	FillCircle(HSI.cx,HSI.cy,HSI.re,config.colorSchema.background); //clear all the compass
	int pex,pey,pix,piy,indexCompass;
	short i;
	for(i=0,indexCompass=dir;i<12;indexCompass+=30,i++) {
		if(indexCompass>359) indexCompass-=360;
		double angle=Deg2Rad(indexCompass);
		pex=HSI.cx;
		pey=HSI.mark_start;
		pix=HSI.cx;
		piy=HSI.major_mark;
		rotatePoint(HSI.cx,HSI.cy,&pex,&pey,angle);
		rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle);
		DrawTwoPointsLine(pex,pey,pix,piy,config.colorSchema.compassRose);
		pix=HSI.cx; //Here we locate the label
		piy=HSI.label_pos;
		rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle);
		FBrenderBlitText(pix-HSI.labelHalfWidth[i],piy-HSI.labelHalfHeight,config.colorSchema.compassRose,config.colorSchema.background,0,HSI.label[i]);
		int index2=indexCompass+5;
		short minor=1,j;
		for(j=0;j<5;index2+=5,j++,minor=!minor) {
			angle=Deg2Rad(index2);
			pex=HSI.cx;
			pey=HSI.mark_start;
			pix=HSI.cx;
			if(minor) piy=HSI.minor_mark;
			else piy=HSI.major_mark;
			rotatePoint(HSI.cx,HSI.cy,&pex,&pey,angle);
			rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle);
			DrawTwoPointsLine(pex,pey,pix,piy,config.colorSchema.compassRose);
		}
	}
}

void drawLabels(int dir) { //also here direction as integer
	int indexLabel;
	short i;
	for(i=0,indexLabel=dir;i<12;indexLabel+=30,i++) {
		if(indexLabel>359) indexLabel-=360;
		double angle=Deg2Rad(indexLabel);
		int pix=HSI.cx; //locate the label
		int piy=HSI.label_pos;
		rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle);
		FBrenderBlitText(pix-HSI.labelHalfWidth[i],piy-HSI.labelHalfHeight,config.colorSchema.compassRose,config.colorSchema.background,0,HSI.label[i]);
	}
}

void drawCDI(double direction, double course, double cdi) { //here we can use double
	if(course<0||course>360) return;
	HSI.actualCourse=course;
	HSI.actualCDI=cdi;
	double angle=course-direction;
	if(angle<0) angle+=360;
	angle=Deg2Rad(angle);
	int pex=HSI.cx-1; //course indicator left
	int pey=HSI.major_mark+2;
	int pix=HSI.cx-1;
	int piy=HSI.cdi_border+1;
	rotatePoint(HSI.cx,HSI.cy,&pex,&pey,angle);
	rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,config.colorSchema.routeIndicator);
	pex=HSI.cx; //course indicator central
	pey=HSI.major_mark;
	pix=HSI.cx;
	piy=HSI.cdi_border;
	rotatePoint(HSI.cx,HSI.cy,&pex,&pey,angle);
	rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,config.colorSchema.routeIndicator);
	pex=HSI.cx+1; //course indicator right
	pey=HSI.major_mark+2;
	pix=HSI.cx+1;
	piy=HSI.cdi_border+1;
	rotatePoint(HSI.cx,HSI.cy,&pex,&pey,angle);
	rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,config.colorSchema.routeIndicator);
	pex=HSI.cx; //first side of the arrow central
	pey=HSI.major_mark;
	pix=HSI.cx-HSI.arrow_side;
	piy=HSI.arrow_end;
	rotatePoint(HSI.cx,HSI.cy,&pex,&pey,angle);
	rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,config.colorSchema.routeIndicator);
	pex=HSI.cx-1; //first side of the right left
	pey=HSI.major_mark+2;
	pix=HSI.cx-HSI.arrow_side-1;
	piy=HSI.arrow_end;
	rotatePoint(HSI.cx,HSI.cy,&pex,&pey,angle);
	rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,config.colorSchema.routeIndicator);
	pex=HSI.cx+1; //other side of the arrow right
	pey=HSI.major_mark+2;
	pix=HSI.cx+HSI.arrow_side+1;
	piy=HSI.arrow_end;
	rotatePoint(HSI.cx,HSI.cy,&pex,&pey,angle);
	rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,config.colorSchema.routeIndicator);
	pex=HSI.cx; //other side of the arrow central
	pey=HSI.major_mark;
	pix=HSI.cx+HSI.arrow_side;
	piy=HSI.arrow_end;
	rotatePoint(HSI.cx,HSI.cy,&pex,&pey,angle);
	rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,config.colorSchema.routeIndicator);
	pex=HSI.cx-1; //other side of the course indicator left
	pey=HSI.major_mark;
	pix=HSI.cx-1;
	piy=HSI.cdi_border;
	rotatePoint(HSI.cx,HSI.cy,&pex,&pey,angle+M_PI);
	rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle+M_PI);
	DrawTwoPointsLine(pex,pey,pix,piy,config.colorSchema.routeIndicator);
	pex=HSI.cx; //other side of the course central
	pey=HSI.major_mark;
	pix=HSI.cx;
	piy=HSI.cdi_border-1;
	rotatePoint(HSI.cx,HSI.cy,&pex,&pey,angle+M_PI);
	rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle+M_PI);
	DrawTwoPointsLine(pex,pey,pix,piy,config.colorSchema.routeIndicator);
	pex=HSI.cx+1; //other side of the course indicator right
	pey=HSI.major_mark;
	pix=HSI.cx+1;
	piy=HSI.cdi_border;
	rotatePoint(HSI.cx,HSI.cy,&pex,&pey,angle+M_PI);
	rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle+M_PI);
	DrawTwoPointsLine(pex,pey,pix,piy,config.colorSchema.routeIndicator);
	short i;
	for(i=-5;i<6;i++) { //ticks of the CDI scale
		pex=pix=HSI.cx+i*HSI.cdi_pixel_tick;
		pey=HSI.cy-HSI.cdi_scale_mark;
		piy=HSI.cy+HSI.cdi_scale_mark;
		rotatePoint(HSI.cx,HSI.cy,&pex,&pey,angle);
		rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle);
		DrawTwoPointsLine(pex,pey,pix,piy,config.colorSchema.cdiScale);
	}
	int dev; //deviation in pixel
	unsigned short cdiColor;
	if(cdi>HSI.cdi_scale) {
		dev=-HSI.cdi_pixel_scale;
		cdiColor=config.colorSchema.caution;
	} else if(cdi<-HSI.cdi_scale) {
		dev=HSI.cdi_pixel_scale;
		cdiColor=config.colorSchema.caution;
	} else {
		dev=-(int)(round((HSI.cdi_pixel_scale*cdi)/HSI.cdi_scale));
		cdiColor=config.colorSchema.cdi;
	}
	pex=HSI.cx+dev-1; //CDI left
	pey=HSI.cdi_border+2;
	pix=HSI.cx+dev-1;
	piy=HSI.cdi_end-1;
	rotatePoint(HSI.cx,HSI.cy,&pex,&pey,angle);
	rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,cdiColor);
	pex=HSI.cx+dev; //CDI center
	pey=HSI.cdi_border+1;
	pix=HSI.cx+dev;
	piy=HSI.cdi_end;
	rotatePoint(HSI.cx,HSI.cy,&pex,&pey,angle);
	rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,cdiColor);
	pex=HSI.cx+dev+1; //CDI right
	pey=HSI.cdi_border+2;
	pix=HSI.cx+dev+1;
	piy=HSI.cdi_end-1;
	rotatePoint(HSI.cx,HSI.cy,&pex,&pey,angle);
	rotatePoint(HSI.cx,HSI.cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,cdiColor);
	DrawTwoPointsLine(HSI.symFuselageL,HSI.symFuselageU,HSI.symFuselageL,HSI.symFuselageD,config.colorSchema.airplaneSymbol); //From here draw the airplane symbol
	DrawTwoPointsLine(HSI.cx,HSI.symFuselageU,HSI.cx,HSI.symFuselageD,config.colorSchema.airplaneSymbol);
	DrawTwoPointsLine(HSI.symFuselageR,HSI.symFuselageU,HSI.symFuselageR,HSI.symFuselageD,config.colorSchema.airplaneSymbol);
	DrawTwoPointsLine(HSI.symWingL,HSI.symWingU,HSI.symWingR,HSI.symWingU,config.colorSchema.airplaneSymbol);
	DrawTwoPointsLine(HSI.symWingL,HSI.symWingC,HSI.symWingR,HSI.symWingC,config.colorSchema.airplaneSymbol);
	DrawTwoPointsLine(HSI.symWingL,HSI.symWingD,HSI.symWingR,HSI.symWingD,config.colorSchema.airplaneSymbol);
	DrawTwoPointsLine(HSI.symTailL,HSI.symTailU,HSI.symTailR,HSI.symTailU,config.colorSchema.airplaneSymbol);
	DrawTwoPointsLine(HSI.symTailL,HSI.symTailC,HSI.symTailR,HSI.symTailC,config.colorSchema.airplaneSymbol);
	DrawTwoPointsLine(HSI.symTailL,HSI.symTailD,HSI.symTailR,HSI.symTailD,config.colorSchema.airplaneSymbol);
}

void diplayCDIvalue(double cdiMt) {
	double cdi=fabs(cdiMt);
	switch(config.trackErrUnit) {
		case FT:
			cdi=m2Ft(cdi);
			if(cdi>=MILE_FT) FBrenderBlitText(5,screen.height-14,config.colorSchema.cdi,config.colorSchema.background,0,"%6.2f Mi",cdi/MILE_FT); //Display in miles
			else FBrenderBlitText(5,screen.height-14,config.colorSchema.cdi,config.colorSchema.background,0,"%5.0f Ft",fabs(cdi));
			break;
		case NM:
			FBrenderBlitText(5,screen.height-14,config.colorSchema.cdi,config.colorSchema.background,0,"%6.2f NM",m2Nm(cdi));
			break;
		case MT:
		default:
			if(cdi>=1000) FBrenderBlitText(5,screen.height-14,config.colorSchema.cdi,config.colorSchema.background,0,"%6.2f Km",cdi/1000); //Display in Km
			else FBrenderBlitText(5,screen.height-14,config.colorSchema.cdi,config.colorSchema.background,0,"%5.0fm",fabs(cdi));
			/* no break */
	}
}

void HSIdraw(double direction, double course, double cdiMt) {
	if(direction<0||direction>360) return;
	int dir=360-HSIround(direction);
	HSI.actualDir=direction;
	if(dir!=HSI.previousDir) { //need to update all the compass
		drawCompass(dir);
		drawCDI(direction,course,cdiMt);
	} else {
		if(course!=HSI.actualCourse || cdiMt!=HSI.actualCDI) {
			FillCircle(HSI.cx,HSI.cy,HSI.cir,config.colorSchema.background); //clear the internal part of the compass
			drawLabels(dir);
			drawCDI(direction,course,cdiMt);
		} //else no need to repaint the HSI
	}
	FBrenderBlitText(5,5,config.colorSchema.dirMarker,config.colorSchema.background,0,"%06.2f",direction);
	//FBrenderBlitText(cx-12,cy-40,config.colorSchema.magneticDir,config.colorSchema.background,0,"%06.2f",actualMagDir);
	diplayCDIvalue(cdiMt);
	FBrenderBlitText(5,20,config.colorSchema.routeIndicator,config.colorSchema.background,0,"%06.2f",course);
	FBrenderFlush();
}

void HSIupdateDir(double direction,double magneticDirection) {
	if(direction<0||direction>360) return;
	int dir=360-(int)round(direction);
	if(dir!=HSI.previousDir) { //need to update all the compass
		drawCompass(dir);
		drawCDI(direction,HSI.actualCourse,HSI.actualCDI);
	} //else no need to repaint the HSI
	HSI.actualDir=direction;
	HSI.actualMagDir=magneticDirection;
	FBrenderBlitText(5,5,config.colorSchema.dirMarker,config.colorSchema.background,0,"%06.2f",direction);
	//FBrenderBlitText(cx-12,cy-40,config.colorSchema.magneticDir,config.colorSchema.background,0,"%06.2f",magneticDirection);
	diplayCDIvalue(HSI.actualCDI);
	FBrenderBlitText(5,20,config.colorSchema.routeIndicator,config.colorSchema.background,0,"%06.2f",HSI.actualCourse);
}

void HSIupdateCDI(double course, double cdi) {
	if(course!=HSI.actualCourse||cdi!=HSI.actualCDI) {
		FillCircle(HSI.cx,HSI.cy,HSI.cir,config.colorSchema.background); //clear the internal part of the compass
		drawLabels(HSI.previousDir);
		drawCDI(HSI.actualDir,course,cdi);
		FBrenderBlitText(5,5,config.colorSchema.dirMarker,config.colorSchema.background,0,"%06.2f",HSI.actualDir);
		//FBrenderBlitText(cx-12,cy-40,config.colorSchema.magneticDir,config.colorSchema.background,0,"%06.2f",actualMagDir);
		diplayCDIvalue(cdi);
		FBrenderBlitText(5,20,config.colorSchema.routeIndicator,config.colorSchema.background,0,"%06.2f",course);
	} //else no need to repaint the HSI
}

void HSIdrawVSIscale(double altFt) {
	long maxScaleFt=(long)altFt;
	if(maxScaleFt==HSI.currentAltFt) return; //no need to update
	HSI.currentAltFt=maxScaleFt;
	FillRect(screen.height+1,1,screen.height+25,screen.height,config.colorSchema.background); //clean all
	maxScaleFt+=HSI.HalfAltScale; //add 500 ft for the top of the scale
	int markerFt=(maxScaleFt/50)*50; //assign the first line altitude
	int markerPx;
	for(markerPx=round((maxScaleFt-markerFt)*0.26)+6;markerPx<HSI.PxAltScale+6&&markerFt>=0;markerPx+=13,markerFt-=50) { //1Ft=0.26Px
		DrawTwoPointsLine(screen.height,markerPx,screen.height+6,markerPx,config.colorSchema.altScale);
		if(markerFt==((markerFt/100)*100)) FBrenderBlitText(screen.height+7,markerPx-4,config.colorSchema.altScale,config.colorSchema.background,0,"%d",(int)(markerFt/100));
	}
	DrawHorizontalLine(screen.height+12,HSI.cy-3,2,config.colorSchema.altMarker);
	DrawHorizontalLine(screen.height+10,HSI.cy-2,4,config.colorSchema.altMarker);
	DrawHorizontalLine(screen.height+8,HSI.cy-1,16,config.colorSchema.altMarker);
	DrawHorizontalLine(screen.height+6,HSI.cy,18,config.colorSchema.altMarker);
	DrawHorizontalLine(screen.height+8,HSI.cy+1,16,config.colorSchema.altMarker);
	DrawHorizontalLine(screen.height+10,HSI.cy+2,4,config.colorSchema.altMarker);
	DrawHorizontalLine(screen.height+12,HSI.cy+3,2,config.colorSchema.altMarker);
	if(HSI.expectedAltFt!=-5000) { //update also VSI
		double newExpectedAltFt=HSI.expectedAltFt;
		HSI.expectedAltFt=-4000; //to force it to be updated
		HSIupdateVSI(newExpectedAltFt);
	}
}

void HSIupdateVSI(double newExpectedAltFt) {
	long expAlt=(long)newExpectedAltFt;
	if(expAlt==HSI.expectedAltFt) return; //no need to update
	HSI.expectedAltFt=expAlt;
	FillRect(screen.height-7,1,screen.height,screen.height,config.colorSchema.background); //clean all
	if(expAlt>HSI.currentAltFt+HSI.HalfAltScale) { //we're too low
		FBrenderPutPixel(screen.height-7,2,config.colorSchema.caution);
		DrawHorizontalLine(screen.height-7,3,3,config.colorSchema.caution);
		DrawHorizontalLine(screen.height-7,4,5,config.colorSchema.caution);
		DrawHorizontalLine(screen.height-7,5,7,config.colorSchema.caution);
		DrawHorizontalLine(screen.height-7,6,5,config.colorSchema.caution);
		DrawHorizontalLine(screen.height-7,7,3,config.colorSchema.caution);
		FBrenderPutPixel(screen.height-7,8,config.colorSchema.caution);
	} else if(expAlt<HSI.currentAltFt-HSI.HalfAltScale) { //we're too high
		FBrenderPutPixel(screen.height-7,screen.height-8,config.colorSchema.caution);
		DrawHorizontalLine(screen.height-7,screen.height-7,3,config.colorSchema.caution);
		DrawHorizontalLine(screen.height-7,screen.height-6,5,config.colorSchema.caution);
		DrawHorizontalLine(screen.height-7,screen.height-5,7,config.colorSchema.caution);
		DrawHorizontalLine(screen.height-7,screen.height-4,5,config.colorSchema.caution);
		DrawHorizontalLine(screen.height-7,screen.height-3,3,config.colorSchema.caution);
		FBrenderPutPixel(screen.height-7,screen.height-2,config.colorSchema.caution);
	} else { //we are on the 1000 Ft scale
		int ypos=round((HSI.currentAltFt+HSI.HalfAltScale-expAlt)*0.26)+6;
		FBrenderPutPixel(screen.height-7,ypos-3,config.colorSchema.vsi);
		DrawHorizontalLine(screen.height-7,ypos-2,3,config.colorSchema.vsi);
		DrawHorizontalLine(screen.height-7,ypos-1,5,config.colorSchema.vsi);
		DrawHorizontalLine(screen.height-7,ypos,7,config.colorSchema.vsi);
		DrawHorizontalLine(screen.height-7,ypos+1,5,config.colorSchema.vsi);
		DrawHorizontalLine(screen.height-7,ypos+2,3,config.colorSchema.vsi);
		FBrenderPutPixel(screen.height-7,ypos+3,config.colorSchema.vsi);
	}
}
