//============================================================================
// Name        : TomTom.h
// Since       : 8/7/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 2/11/2013
// Description : I/O TomTom interface
//============================================================================

#ifndef TOMTOM_H_
#define TOMTOM_H_

#define FB_DEVICE_NAME "/dev/ts"
#define CharWidth    8

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
void TsScreen_Init();
void TsScreen_Exit();
int TsScreen_pen(int *x, int *y, int *pen);
int TsScreen_touch(int *x, int *y);
unsigned long FixSqrt(unsigned long x);
void FbRender_PutPixel(int x, int y, unsigned short color);
void DrawHorizontalLine(int X, int Y, int width, unsigned short color);
void FillCircle(int cx, int cy, int aRad, unsigned short color);
void PutLinePoint(int x, int y, unsigned short color, int width);
short checkBattery(short *batVolt, short *refVolt, short *chargeCurr);
short enableGPS();
void disableGPS();
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

#endif /* TOMTOM_H_ */
