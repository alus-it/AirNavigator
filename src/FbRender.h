//============================================================================
// Name        : FbRender.h
// Since       : 8/7/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 3/11/2013
// Description : Header of FbRender.c the FrameBuffer renderer
//============================================================================

#ifndef FBRENDER_H_
#define FBRENDER_H_

struct screenConfig {
	int height,width; //size of screen in pixel
};

struct colorConfig {
	unsigned short background;
	unsigned short compassRose;
	unsigned short dirMarker;
	unsigned short magneticDir;
	unsigned short routeIndicator;
	unsigned short cdi;
	unsigned short cdiScale;
	unsigned short altScale;
	unsigned short vsi;
	unsigned short altMarker;
	unsigned short text;
	unsigned short warning;
};

struct screenConfig screen;

struct colorConfig colorSchema;

inline unsigned short Color(int r, int g, int b);
int FbRender_Bpp();
void FbRender_Flush();
short FbRender_Open();
void FbRender_Close();
void FbRender_Clear(int aFromY, int aNrLines, unsigned short aColor);
void FbRender_BlitCharacter(int x, int y, unsigned short aColor, unsigned short aBackColor, char character);
void FbRender_BlitCharacterItalic(int x, int y, unsigned short aColor, unsigned short aBackColor, char character);
int FbRender_BlitText(int x, int y, unsigned short aColor, unsigned short aBackColor, unsigned short italic, const char *args, ...);
void FbRender_PutPixel(int x, int y, unsigned short color);
void DrawHorizontalLine(int X, int Y, int width, unsigned short color);
void FillCircle(int cx, int cy, int aRad, unsigned short color);
void PutLinePoint(int x, int y, unsigned short color, int width);
void DrawTwoPointsLine(int ax, int ay, int bx, int by, unsigned short color);
void FillRect(int ulx, int uly, int drx, int dry, unsigned short color);
void PrintPosition(int latD, int latM, double latS, short N, int lonD, int lonM, double lonS, short E);
void PrintSpeed(double speedKmh, double speedKnots);
void PrintAltitude(double altMt, double altFt);
void PrintVerticalSpeed(double FtMin);
void PrintTurnRate(double DegMin);
void PrintDate(int day, int month, int year);
void PrintTime(int hour, int minute, float second);
void PrintFixMode(int fixMode);
void PrintNumOfSats(int activeSats, int satsInView);
void PrintDiluitions(float pDiluition, float hDiluition, float vDiluition);
void PrintNavStatus(int status, char *WPname);
void PrintNavTrackATD(double atdRad);
void PrintNavRemainingDistWP(double dist, double averageSpeed, double hours);
void PrintNavRemainingDistDST(double dist, double averageSpeed, double hours);
void PrintNavDTG(double distRad);

#endif /* FBRENDER_H_ */
