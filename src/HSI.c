//============================================================================
// Name        : HSI.c
// Since       : 16/9/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 12/11/2011
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

char *labels[]={"N","03","06","E","12","15","S","21","24","W","30","33"};
int labelHalfWidth[]={2,6,6,2,6,6,2,6,6,2,6,6};
int labelHalfHeight=4;

void HSIinitialize(double directionDeg, double courseDeg, double courseDeviationMt) {
	cx=screen.height/2; //aligned on the left
	cy=screen.height/2; //half height
	mark_start=10;
	re=cy-mark_start+1; //external radius
	major_mark=mark_start+20;
	label_pos=major_mark+10;
	dir_display_pos=label_pos+15;
	ri=cy-mark_start-major_mark; //internal radius
	cir=ri+11; //clear internal radius used to clear the CDI but not the external compass
	minor_mark=mark_start+12;
	arrow_end=major_mark+18;
	arrow_side=8;
	cdi_border=major_mark+31; //ri-ri*sen(45°)=31
	cdi_end=screen.height-cdi_border;
	cdi_pixel_scale=75; //ri*sen(45°)=75
	cdi_pixel_tick=cdi_pixel_scale/5;
	cdi_scale_mark=15;
	cdi_scale=500; // 0.3NM = 0.56 Km
	previousDir=-456;
	actualMagDir=0;
	actualCourse=-456;
	actualCDI=0;
	currentAltFt=-5000;
	expectedAltFt=-5000;
	HalfAltScale=500;
	if(screen.height==240) HalfAltScale=438;
	PxAltScale=screen.height-12;
	DrawTwoPointsLine(cx-1,0,cx-1,mark_start-4,colorSchema.dirMarker);
	DrawTwoPointsLine(cx,0,cx,mark_start,colorSchema.dirMarker);
	DrawTwoPointsLine(cx+1,0,cx+1,mark_start-4,colorSchema.dirMarker);
	HSIdraw(directionDeg,courseDeg,courseDeviationMt);
	DrawTwoPointsLine(screen.height,6,screen.height,screen.height-6,colorSchema.altScale); //the line of the altitude scale
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

int calcPixelDeviation(double courseDevMt) {
	if(courseDevMt>cdi_scale) return -cdi_pixel_scale;
	if(courseDevMt<-cdi_scale) return cdi_pixel_scale;
	return -(int)(round((cdi_pixel_scale*courseDevMt)/cdi_scale));
}

void drawCompass(int dir) { //works with direction as integer
	previousDir=dir;
	FillCircle(cx,cy,re,colorSchema.background); //clear all the compass
	int pex,pey,pix,piy,index;
	short i;
	for(i=0,index=dir;i<12;index+=30,i++) {
		if(index>359) index-=360;
		double angle=Deg2Rad(index);
		pex=cx;
		pey=mark_start;
		pix=cx;
		piy=major_mark;
		rotatePoint(cx,cy,&pex,&pey,angle);
		rotatePoint(cx,cy,&pix,&piy,angle);
		DrawTwoPointsLine(pex,pey,pix,piy,colorSchema.compassRose);
		pix=cx; //Here we locate the label
		piy=label_pos;
		rotatePoint(cx,cy,&pix,&piy,angle);
		FBrenderBlitText(pix-labelHalfWidth[i],piy-labelHalfHeight,colorSchema.compassRose,colorSchema.background,0,labels[i]);
		int index2=index+5;
		short minor=1,j;
		for(j=0;j<5;index2+=5,j++,minor=!minor) {
			angle=Deg2Rad(index2);
			pex=cx;
			pey=mark_start;
			pix=cx;
			if(minor) piy=minor_mark;
			else piy=major_mark;
			rotatePoint(cx,cy,&pex,&pey,angle);
			rotatePoint(cx,cy,&pix,&piy,angle);
			DrawTwoPointsLine(pex,pey,pix,piy,colorSchema.compassRose);
		}
	}
}

void drawLabels(int dir) { //also here direction as integer
	int index;
	short i;
	for(i=0,index=dir;i<12;index+=30,i++) {
		if(index>359) index-=360;
		double angle=Deg2Rad(index);
		int pix=cx; //locate the label
		int piy=label_pos;
		rotatePoint(cx,cy,&pix,&piy,angle);
		FBrenderBlitText(pix-labelHalfWidth[i],piy-labelHalfHeight,colorSchema.compassRose,colorSchema.background,0,labels[i]);
	}
}

void drawCDI(double direction, double course, double cdi) { //here we can use double
	if(course<0||course>360) return;
	actualCourse=course;
	actualCDI=cdi;
	double angle=course-direction;
	if(angle<0) angle+=360;
	angle=Deg2Rad(angle);
	int pex=cx-1; //course indicator left
	int pey=major_mark+2;
	int pix=cx-1;
	int piy=cdi_border+1;
	rotatePoint(cx,cy,&pex,&pey,angle);
	rotatePoint(cx,cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,colorSchema.routeIndicator);
	pex=cx; //course indicator central
	pey=major_mark;
	pix=cx;
	piy=cdi_border;
	rotatePoint(cx,cy,&pex,&pey,angle);
	rotatePoint(cx,cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,colorSchema.routeIndicator);
	pex=cx+1; //course indicator right
	pey=major_mark+2;
	pix=cx+1;
	piy=cdi_border+1;
	rotatePoint(cx,cy,&pex,&pey,angle);
	rotatePoint(cx,cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,colorSchema.routeIndicator);
	pex=cx; //first side of the arrow central
	pey=major_mark;
	pix=cx-arrow_side;
	piy=arrow_end;
	rotatePoint(cx,cy,&pex,&pey,angle);
	rotatePoint(cx,cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,colorSchema.routeIndicator);
	pex=cx-1; //first side of the right left
	pey=major_mark+2;
	pix=cx-arrow_side-1;
	piy=arrow_end;
	rotatePoint(cx,cy,&pex,&pey,angle);
	rotatePoint(cx,cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,colorSchema.routeIndicator);
	pex=cx+1; //other side of the arrow right
	pey=major_mark+2;
	pix=cx+arrow_side+1;
	piy=arrow_end;
	rotatePoint(cx,cy,&pex,&pey,angle);
	rotatePoint(cx,cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,colorSchema.routeIndicator);
	pex=cx; //other side of the arrow central
	pey=major_mark;
	pix=cx+arrow_side;
	piy=arrow_end;
	rotatePoint(cx,cy,&pex,&pey,angle);
	rotatePoint(cx,cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,colorSchema.routeIndicator);
	pex=cx-1; //other side of the course indicator left
	pey=major_mark;
	pix=cx-1;
	piy=cdi_border;
	rotatePoint(cx,cy,&pex,&pey,angle+M_PI);
	rotatePoint(cx,cy,&pix,&piy,angle+M_PI);
	DrawTwoPointsLine(pex,pey,pix,piy,colorSchema.routeIndicator);
	pex=cx; //other side of the course central
	pey=major_mark;
	pix=cx;
	piy=cdi_border-1;
	rotatePoint(cx,cy,&pex,&pey,angle+M_PI);
	rotatePoint(cx,cy,&pix,&piy,angle+M_PI);
	DrawTwoPointsLine(pex,pey,pix,piy,colorSchema.routeIndicator);
	pex=cx+1; //other side of the course indicator right
	pey=major_mark;
	pix=cx+1;
	piy=cdi_border;
	rotatePoint(cx,cy,&pex,&pey,angle+M_PI);
	rotatePoint(cx,cy,&pix,&piy,angle+M_PI);
	DrawTwoPointsLine(pex,pey,pix,piy,colorSchema.routeIndicator);
	short i;
	for(i=-5;i<6;i++) { //ticks of the CDI scale
		pex=pix=cx+i*cdi_pixel_tick;
		pey=cy-cdi_scale_mark;
		piy=cy+cdi_scale_mark;
		rotatePoint(cx,cy,&pex,&pey,angle);
		rotatePoint(cx,cy,&pix,&piy,angle);
		DrawTwoPointsLine(pex,pey,pix,piy,colorSchema.cdiScale);
	}
	int dev=calcPixelDeviation(cdi); //Course Deviation Indicator
	pex=cx+dev-1; //CDI left
	pey=cdi_border+2;
	pix=cx+dev-1;
	piy=cdi_end-1;
	rotatePoint(cx,cy,&pex,&pey,angle);
	rotatePoint(cx,cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,colorSchema.cdi);
	pex=cx+dev; //CDI center
	pey=cdi_border+1;
	pix=cx+dev;
	piy=cdi_end;
	rotatePoint(cx,cy,&pex,&pey,angle);
	rotatePoint(cx,cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,colorSchema.cdi);
	pex=cx+dev+1; //CDI right
	pey=cdi_border+2;
	pix=cx+dev+1;
	piy=cdi_end-1;
	rotatePoint(cx,cy,&pex,&pey,angle);
	rotatePoint(cx,cy,&pix,&piy,angle);
	DrawTwoPointsLine(pex,pey,pix,piy,colorSchema.cdi);
}

void diplayCDIvalue(double cdiMt) {
	double cdi=fabs(cdiMt);
	switch(config.trackErrUnit) {
		case FT:
			cdi=m2Ft(cdi);
			if(cdi>=MILE_FT) FBrenderBlitText(5,screen.height-14,colorSchema.cdi,colorSchema.background,0,"%6.2f Mi",cdi/MILE_FT); //Display in miles
			else FBrenderBlitText(5,screen.height-14,colorSchema.cdi,colorSchema.background,0,"%5.0f Ft",fabs(cdi));
			break;
		case NM:
			FBrenderBlitText(5,screen.height-14,colorSchema.cdi,colorSchema.background,0,"%6.2f NM",m2Nm(cdi));
			break;
		case MT:
		default:
			if(cdi>=1000) FBrenderBlitText(5,screen.height-14,colorSchema.cdi,colorSchema.background,0,"%6.2f Km",cdi/1000); //Display in Km
			else FBrenderBlitText(5,screen.height-14,colorSchema.cdi,colorSchema.background,0,"%5.0fm",fabs(cdi));
			/* no break */
	}
}

void HSIdraw(double direction, double course, double cdiMt) {
	if(direction<0||direction>360) return;
	int dir=360-HSIround(direction);
	actualDir=direction;
	if(dir!=previousDir) { //need to update all the compass
		drawCompass(dir);
		drawCDI(direction,course,cdiMt);
	} else {
		if(course!=actualCourse || cdiMt!=actualCDI) {
			FillCircle(cx,cy,cir,colorSchema.background); //clear the internal part of the compass
			drawLabels(dir);
			drawCDI(direction,course,cdiMt);
		} //else no need to repaint the HSI
	}
	FBrenderBlitText(5,5,colorSchema.dirMarker,colorSchema.background,0,"%06.2f",direction);
	//FBrenderBlitText(cx-12,cy-40,colorSchema.magneticDir,colorSchema.background,0,"%06.2f",actualMagDir);
	diplayCDIvalue(cdiMt);
	FBrenderBlitText(5,20,colorSchema.routeIndicator,colorSchema.background,0,"%06.2f",course);
	FBrenderFlush();
}

void HSIupdateDir(double direction,double magneticDirection) {
	if(direction<0||direction>360) return;
	int dir=360-(int)round(direction);
	if(dir!=previousDir) { //need to update all the compass
		drawCompass(dir);
		drawCDI(direction,actualCourse,actualCDI);
	} //else no need to repaint the HSI
	actualDir=direction;
	actualMagDir=magneticDirection;
	FBrenderBlitText(5,5,colorSchema.dirMarker,colorSchema.background,0,"%06.2f",direction);
	//FBrenderBlitText(cx-12,cy-40,colorSchema.magneticDir,colorSchema.background,0,"%06.2f",magneticDirection);
	diplayCDIvalue(actualCDI);
	FBrenderBlitText(5,20,colorSchema.routeIndicator,colorSchema.background,0,"%06.2f",actualCourse);
}

void HSIupdateCDI(double course, double cdi) {
	if(course!=actualCourse||cdi!=actualCDI) {
		FillCircle(cx,cy,cir,colorSchema.background); //clear the internal part of the compass
		drawLabels(previousDir);
		drawCDI(actualDir,course,cdi);
		FBrenderBlitText(5,5,colorSchema.dirMarker,colorSchema.background,0,"%06.2f",actualDir);
		//FBrenderBlitText(cx-12,cy-40,colorSchema.magneticDir,colorSchema.background,0,"%06.2f",actualMagDir);
		diplayCDIvalue(cdi);
		FBrenderBlitText(5,20,colorSchema.routeIndicator,colorSchema.background,0,"%06.2f",course);
	} //else no need to repaint the HSI
}

void HSIdrawVSIscale(double altFt) {
	long maxScaleFt=(long)altFt;
	if(maxScaleFt==currentAltFt) return; //no need to update
	currentAltFt=maxScaleFt;
	FillRect(screen.height+1,1,screen.height+25,screen.height,colorSchema.background); //clean all
	maxScaleFt+=HalfAltScale; //add 500 ft for the top of the scale
	int markerFt=(maxScaleFt/50)*50; //assign the first line altitude
	int markerPx;
	for(markerPx=round((maxScaleFt-markerFt)*0.26)+6;markerPx<PxAltScale+6&&markerFt>=0;markerPx+=13,markerFt-=50) { //1Ft=0.26Px
		DrawTwoPointsLine(screen.height,markerPx,screen.height+6,markerPx,colorSchema.altScale);
		if(markerFt==((markerFt/100)*100)) FBrenderBlitText(screen.height+7,markerPx-4,colorSchema.altScale,colorSchema.background,0,"%d",(int)(markerFt/100));
	}
	DrawHorizontalLine(screen.height+12,cy-3,2,colorSchema.altMarker);
	DrawHorizontalLine(screen.height+10,cy-2,4,colorSchema.altMarker);
	DrawHorizontalLine(screen.height+8,cy-1,16,colorSchema.altMarker);
	DrawHorizontalLine(screen.height+6,cy,18,colorSchema.altMarker);
	DrawHorizontalLine(screen.height+8,cy+1,16,colorSchema.altMarker);
	DrawHorizontalLine(screen.height+10,cy+2,4,colorSchema.altMarker);
	DrawHorizontalLine(screen.height+12,cy+3,2,colorSchema.altMarker);
	if(expectedAltFt!=-5000) { //update also VSI
		double newExpectedAltFt=expectedAltFt;
		expectedAltFt=-4000; //to force it to be updated
		HSIupdateVSI(newExpectedAltFt);
	}
}

void HSIupdateVSI(double newExpectedAltFt) {
	long expAlt=(long)newExpectedAltFt;
	if(expAlt==expectedAltFt) return; //no need to update
	expectedAltFt=expAlt;
	FillRect(screen.height-7,1,screen.height,screen.height,colorSchema.background); //clean all
	if(expAlt>currentAltFt+HalfAltScale) { //we're too low
		FBrenderPutPixel(screen.height-7,2,colorSchema.warning);
		DrawHorizontalLine(screen.height-7,3,3,colorSchema.warning);
		DrawHorizontalLine(screen.height-7,4,5,colorSchema.warning);
		DrawHorizontalLine(screen.height-7,5,7,colorSchema.warning);
		DrawHorizontalLine(screen.height-7,6,5,colorSchema.warning);
		DrawHorizontalLine(screen.height-7,7,3,colorSchema.warning);
		FBrenderPutPixel(screen.height-7,8,colorSchema.warning);
	} else if(expAlt<currentAltFt-HalfAltScale) { //we're too high
		FBrenderPutPixel(screen.height-7,screen.height-8,colorSchema.warning);
		DrawHorizontalLine(screen.height-7,screen.height-7,3,colorSchema.warning);
		DrawHorizontalLine(screen.height-7,screen.height-6,5,colorSchema.warning);
		DrawHorizontalLine(screen.height-7,screen.height-5,7,colorSchema.warning);
		DrawHorizontalLine(screen.height-7,screen.height-4,5,colorSchema.warning);
		DrawHorizontalLine(screen.height-7,screen.height-3,3,colorSchema.warning);
		FBrenderPutPixel(screen.height-7,screen.height-2,colorSchema.warning);
	} else { //we are on the 1000 Ft scale
		int ypos=round((currentAltFt+HalfAltScale-expAlt)*0.26)+6;
		FBrenderPutPixel(screen.height-7,ypos-3,colorSchema.vsi);
		DrawHorizontalLine(screen.height-7,ypos-2,3,colorSchema.vsi);
		DrawHorizontalLine(screen.height-7,ypos-1,5,colorSchema.vsi);
		DrawHorizontalLine(screen.height-7,ypos,7,colorSchema.vsi);
		DrawHorizontalLine(screen.height-7,ypos+1,5,colorSchema.vsi);
		DrawHorizontalLine(screen.height-7,ypos+2,3,colorSchema.vsi);
		FBrenderPutPixel(screen.height-7,ypos+3,colorSchema.vsi);
	}
}
