//============================================================================
// Name        : FBrender.c
// Since       : 9/7/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2013 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 11/12/2013
// Description : FrameBuffer renderer
//============================================================================

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/fb.h>
#include "FBrender.h"
#include "Navigator.h"
#include "AirCalc.h"
#include "GPSreceiver.h"
#include "Configuration.h"

#define CHAR_WIDTH    8

struct FBrenderStruct {
	int fbfd;
	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;
	long screensize;
	char *fbp;
	char *fbbackp;
	int iClipTop,iClipBottom,iClipMin,iClipMax;
	short isOpen;
};

void FBrenderScroll(int target_y, int source_y, int height);
unsigned long FixSqrt(unsigned long x);

static struct FBrenderStruct FBrender = {
	.fbfd=-1,
	.screensize=0,
	.fbp=0,
	.fbbackp=0,
	.iClipTop=0,
	.iClipBottom=272,
	.iClipMin=0,
	.iClipMax=480,
	.isOpen=-1
};

unsigned short Color(int r, int g, int b) {
	return (b>>3)+((g>>2)*32)+((r>>3)*2048);// (gray>>2)<<5;
}

int FBrenderBpp(void) {
	return FBrender.vinfo.bits_per_pixel;
}

short FBrenderOpen(void) {
	FBrender.fbfd=open("/dev/fb",O_RDWR);
	if(!FBrender.fbfd) { //open the framebuffer
		printf("FATAL ERROR: Impossible to open the framebuffer.\n");
		FBrender.isOpen=-1;
		return 0;
	}
	if(ioctl(FBrender.fbfd,FBIOGET_FSCREENINFO,&FBrender.finfo)) { // Get fixed screen information
		printf("FATAL ERROR: Impossible to get fixed screen information from framebuffer.\n");
		close(FBrender.fbfd);
		FBrender.isOpen=-2;
		return 0;
	}
	if(ioctl(FBrender.fbfd,FBIOGET_VSCREENINFO,&FBrender.vinfo)) { // Get variable screen information
		printf("FATAL ERROR: Impossible to get variable screen information from framebuffer.\n");
		close(FBrender.fbfd);
		FBrender.isOpen=-3;
		return 0;
	}
	screen.width=FBrender.vinfo.xres;
	screen.height=FBrender.vinfo.yres;
	FBrender.iClipTop=0;
	FBrender.iClipBottom=screen.height;
	FBrender.iClipMin=0;
	FBrender.iClipMax=screen.width;
	FBrender.screensize=FBrender.vinfo.xres*FBrender.vinfo.yres*FBrender.vinfo.bits_per_pixel/8; // Figure out the size of the screen in bytes
	FBrender.fbp=(char *)mmap(0,FBrender.screensize,PROT_READ|PROT_WRITE,MAP_SHARED,FBrender.fbfd,0); // Map the device to memory
	if((int)FBrender.fbp==-1) {
		printf("FATAL ERROR: Impossible to map the screen device to memory.\n");
		munmap(FBrender.fbp,FBrender.screensize);
		close(FBrender.fbfd);
		FBrender.isOpen=-4;
		return 0;
	}
	FBrender.fbbackp=(char*)malloc(FBrender.screensize);
	FBrender.isOpen=1;
	return 1;
}

void FBrenderClose(void) {
	if(FBrender.isOpen!=1) return;
	if(FBrender.fbfd>0) {
		munmap(FBrender.fbp,FBrender.screensize);
		close(FBrender.fbfd);
	}
	if(FBrender.fbbackp) free(FBrender.fbbackp);
	FBrender.fbfd=-1;
}

void FBrenderClear(int aFromY, int aNrLines, unsigned short aColor) {
	unsigned short *ptr=(unsigned short*)(FBrender.fbbackp+aFromY*((FBrender.vinfo.xres*FBrender.vinfo.bits_per_pixel)/8));
	unsigned short *endp=ptr+aNrLines*((FBrender.vinfo.xres*FBrender.vinfo.bits_per_pixel)/16);
	while(ptr<endp) {
		*ptr++=aColor;
		*ptr++=aColor;
		*ptr++=aColor;
		*ptr++=aColor;
	}
}

inline void FBrenderPutPixel(int x, int y, unsigned short color) {
	unsigned short *ptr=(unsigned short*)(FBrender.fbbackp+y*((FBrender.vinfo.xres*FBrender.vinfo.bits_per_pixel)/8));
	ptr+=x;
	*ptr=color;
}

// Character set
static const unsigned char asciiTable[]={0x00,0x00,0x00,0x00,0x00,//0x00,
		0x00,0x00,0x5F,0x00,0x00,//0x00,
		0x00,0x07,0x00,0x07,0x00,//0x00,
		0x14,0x7F,0x14,0x7F,0x14,//0x00,
		0x24,0x2A,0x7F,0x2A,0x12,//0x00,
		0x23,0x13,0x08,0x64,0x62,//0x00,
		0x36,0x49,0x55,0x22,0x50,//0x00,
		0x00,0x05,0x03,0x00,0x00,//0x00,
		0x00,0x1C,0x22,0x41,0x00,//0x00,
		0x00,0x41,0x22,0x1C,0x00,//0x00,
		0x14,0x08,0x3E,0x08,0x14,//0x00,
		0x08,0x08,0x3E,0x08,0x08,//0x00,
		0x00,0x50,0x30,0x00,0x00,//0x00,
		0x08,0x08,0x08,0x08,0x08,//0x00,
		0x00,0x60,0x60,0x00,0x00,//0x00,
		0x20,0x10,0x08,0x04,0x02,//0x00,
		0x3E,0x51,0x49,0x45,0x3E,//0x00,
		0x00,0x42,0x7F,0x40,0x00,//0x00,
		0x42,0x61,0x51,0x49,0x46,//0x00,
		0x21,0x41,0x45,0x4B,0x31,//0x00,
		0x18,0x14,0x12,0x7F,0x10,//0x00,
		0x27,0x45,0x45,0x45,0x39,//0x00,
		0x3C,0x4A,0x49,0x49,0x30,//0x00,
		0x01,0x71,0x09,0x05,0x03,//0x00,
		0x36,0x49,0x49,0x49,0x36,//0x00,
		0x06,0x49,0x49,0x29,0x1E,//0x00,
		0x00,0x36,0x36,0x00,0x00,//0x00,
		0x00,0x56,0x36,0x00,0x00,//0x00,
		0x08,0x14,0x22,0x41,0x00,//0x00,
		0x14,0x14,0x14,0x14,0x14,//0x00,
		0x00,0x41,0x22,0x14,0x08,//0x00,
		0x02,0x01,0x51,0x09,0x06,//0x00,
		0x32,0x49,0x79,0x41,0x3E,//0x00,
		0x7E,0x11,0x11,0x11,0x7E,//0x00,
		0x7F,0x49,0x49,0x49,0x36,//0x00,
		0x3E,0x41,0x41,0x41,0x22,//0x00,
		0x7F,0x41,0x41,0x22,0x1C,//0x00,
		0x7F,0x49,0x49,0x49,0x41,//0x00,
		0x7F,0x09,0x09,0x09,0x01,//0x00,
		0x3E,0x41,0x49,0x49,0x7A,//0x00,
		0x7F,0x08,0x08,0x08,0x7F,//0x00,
		0x00,0x41,0x7F,0x41,0x00,//0x00,
		0x20,0x40,0x41,0x3F,0x01,//0x00,
		0x7F,0x08,0x14,0x22,0x41,//0x00,
		0x7F,0x40,0x40,0x40,0x40,//0x00,
		0x7F,0x02,0x0C,0x02,0x7F,//0x00,
		0x7F,0x04,0x08,0x10,0x7F,//0x00,
		0x3E,0x41,0x41,0x41,0x3E,//0x00,
		0x7F,0x09,0x09,0x09,0x06,//0x00,
		0x3E,0x41,0x51,0x21,0x5E,//0x00,
		0x7F,0x09,0x19,0x29,0x46,//0x00,
		0x46,0x49,0x49,0x49,0x31,//0x00,
		0x01,0x01,0x7F,0x01,0x01,//0x00,
		0x3F,0x40,0x40,0x40,0x3F,//0x00,
		0x1F,0x20,0x40,0x20,0x1F,//0x00,
		0x3F,0x40,0x38,0x40,0x3F,//0x00,
		0x63,0x14,0x08,0x14,0x63,//0x00,
		0x07,0x08,0x70,0x08,0x07,//0x00,
		0x61,0x51,0x49,0x45,0x43,//0x00,
		0x00,0x7F,0x41,0x41,0x00,//0x00,
		0x02,0x04,0x08,0x10,0x20,//0x00,
		0x00,0x41,0x41,0x7F,0x00,//0x00,
		0x04,0x02,0x01,0x02,0x04,//0x00,
		0x40,0x40,0x40,0x40,0x40,//0x40,
		0x00,0x01,0x02,0x04,0x00,//0x00,
		0x20,0x54,0x54,0x54,0x78,//0x00,
		0x7F,0x48,0x44,0x44,0x38,//0x00,
		0x38,0x44,0x44,0x44,0x20,//0x00,
		0x38,0x44,0x44,0x48,0x7F,//0x00,
		0x38,0x54,0x54,0x54,0x18,//0x00,
		0x08,0x7E,0x09,0x01,0x02,//0x00,
		0x0C,0x52,0x52,0x52,0x3E,//0x00,
		0x7F,0x08,0x04,0x04,0x78,//0x00,
		0x00,0x44,0x7D,0x40,0x00,//0x00,
		0x20,0x40,0x44,0x3D,0x00,//0x00,
		0x7F,0x10,0x28,0x44,0x00,//0x00,
		0x00,0x41,0x7F,0x40,0x00,//0x00,
		0x7C,0x04,0x18,0x04,0x78,//0x00,
		0x7C,0x08,0x04,0x04,0x78,//0x00,
		0x38,0x44,0x44,0x44,0x38,//0x00,
		0x7C,0x14,0x14,0x14,0x08,//0x00,
		0x08,0x14,0x14,0x18,0x7C,//0x00,
		0x7C,0x08,0x04,0x04,0x08,//0x00,
		0x48,0x54,0x54,0x54,0x20,//0x00,
		0x04,0x3F,0x44,0x40,0x20,//0x00,
		0x3C,0x40,0x40,0x20,0x7C,//0x00,
		0x1C,0x20,0x40,0x20,0x1C,//0x00,
		0x3C,0x40,0x30,0x40,0x3C,//0x00,
		0x44,0x28,0x10,0x28,0x44,//0x00,
		0x0C,0x50,0x50,0x50,0x3C,//0x00,
		0x44,0x64,0x54,0x4C,0x44,//0x00,
		0x00,0x08,0x36,0x41,0x00,//0x00,
		0x00,0x00,0x7F,0x00,0x00,//0x00,
		0x00,0x41,0x36,0x08,0x00,//0x00,
		0x10,0x08,0x08,0x10,0x08,//0x00,
		0x78,0x46,0x41,0x46,0x78,//0x00
		};

void FBrenderBlitCharacter(int x, int y, unsigned short aColor, unsigned short aBackColor, char character) {
	unsigned char data0,data1,data2,data3,data4;
	if(x<0||y<0) return;
	if(x>screen.width-CHAR_WIDTH-1||y>screen.height-7) return;
	if((character<32)||(character>126)) character=' ';
	const unsigned char *aptr=&asciiTable[(character-32)*5];
	data0=*aptr++;
	data1=*aptr++;
	data2=*aptr++;
	data3=*aptr++;
	data4=*aptr;
	register unsigned short *ptr=(unsigned short*)(FBrender.fbbackp+y*((FBrender.vinfo.xres*FBrender.vinfo.bits_per_pixel)/8));
	ptr+=x;
	register unsigned short *endp=ptr+CHAR_WIDTH*screen.width;
	while(ptr<endp) {
		if(data0&1) *ptr++=aColor;
		else *ptr++=aBackColor;
		data0>>=1;
		if(data1&1) *ptr++=aColor;
		else *ptr++=aBackColor;
		data1>>=1;
		if(data2&1) *ptr++=aColor;
		else *ptr++=aBackColor;
		data2>>=1;
		if(data3&1) *ptr++=aColor;
		else *ptr++=aBackColor;
		data3>>=1;
		if(data4&1) *ptr++=aColor;
		else *ptr++=aBackColor;
		data4>>=1;
		ptr+=screen.width-(CHAR_WIDTH-3);
	}
}

void FBrenderBlitCharacterItalic(int x, int y, unsigned short aColor, unsigned short aBackColor, char character) {
	unsigned char data0,data1,data2,data3,data4;
	if(x<0||y<0) return;
	if(x>screen.width-CHAR_WIDTH-1||y>screen.height-7) return;
	if((character<32)||(character>126)) character=' ';
	const unsigned char *aptr=&asciiTable[(character-32)*5];
	data0=*aptr++;
	data1=*aptr++;
	data2=*aptr++;
	data3=*aptr++;
	data4=*aptr;
	register unsigned short *ptr=(unsigned short*)(FBrender.fbbackp+y*((FBrender.vinfo.xres*FBrender.vinfo.bits_per_pixel)/8));
	ptr+=x;
	register unsigned short *endp=ptr+CHAR_WIDTH*screen.width;
	while(ptr<endp) {
		if(data0&1) *ptr++=aColor;
		else *ptr++=aBackColor;
		data0>>=1;
		if(data1&1) *ptr++=aColor;
		else *ptr++=aBackColor;
		data1>>=1;
		if(data2&1) *ptr++=aColor;
		else *ptr++=aBackColor;
		data2>>=1;
		if(data3&1) *ptr++=aColor;
		else *ptr++=aBackColor;
		data3>>=1;
		if(data4&1) *ptr++=aColor;
		else *ptr++=aBackColor;
		data4>>=1;
		ptr+=screen.width-(CHAR_WIDTH-2);
	}
}

int FBrenderBlitText(int x, int y, unsigned short aColor, unsigned short aBackColor, bool italic, const char *args, ...) {
	int done=-1;
	if(args!=NULL) {
		va_list arg;
		va_start(arg,args);
		char *str;
		done=vasprintf(&str,args,arg);
		if(done) {
			int i=0;
			while(str[i]) {
				if(italic) FBrenderBlitCharacterItalic(x,y,aColor,aBackColor,str[i]);
				else FBrenderBlitCharacter(x,y,aColor,aBackColor,str[i]);
				x+=CHAR_WIDTH;
				i++;
			}
		}
		free(str);
	}
	return done;
}

void FBrenderFlush(void) {
	memcpy(FBrender.fbp,FBrender.fbbackp,FBrender.screensize);
}

void FBrenderScroll(int target_y, int source_y, int height) {
	memmove(FBrender.fbbackp+target_y*screen.height*2,FBrender.fbbackp+source_y*screen.height*2,height*screen.height*2);
}

unsigned long FixSqrt(unsigned long x) {
	unsigned long r,nr,m;
	r=0;
	m=0x40000000;
	do {
		nr=r+m;
		if(nr<=x) {
			x-=nr;
			r=nr+m;
		}
		r>>=1;
		m>>=2;
	} while(m!=0);
	if(x>r) r++;
	return r;
}

void DrawHorizontalLine(int X, int Y, int width, unsigned short color) {
	register int w=width; // in pixels
	if(Y<FBrender.iClipTop) return;
	if(Y>=FBrender.iClipBottom) return;
	if(FBrender.iClipMin-X>0) {// clip left margin
		w-=(FBrender.iClipMin-X);
		X=FBrender.iClipMin;
	}
	if(w>FBrender.iClipMax-X) w=FBrender.iClipMax-X; // clip right margin
	while(w-->0) FBrenderPutPixel(X++,Y,color); // put the pixels...
}

void FillCircle(int cx, int cy, int aRad, unsigned short color) {
	int y;
	for(y=cy-aRad;y<=cy+aRad;++y) {
		register unsigned long tmp;
		tmp=aRad*aRad-(y-cy)*(y-cy);
		tmp=FixSqrt(tmp);
		DrawHorizontalLine(cx-tmp,y,tmp<<1,color);
	}
}

void PutLinePoint(int x, int y, unsigned short color, int width) {
	if(width>1) FillCircle(x,y,width,color);
	else FBrenderPutPixel(x,y,color);
}

void DrawTwoPointsLine(int ax, int ay, int bx, int by, unsigned short color) {
	if(ax!=bx) { //non vertical line
		double m=(double)(by-ay)/(double)(bx-ax);
		double q=ay-m*ax;
		if(m<-1 || m>1) {
			int y;
			if(by>ay) for(y=ay;y<by;y++) FBrenderPutPixel(((int)round((y-q)/m)),y,color);
			else for(y=by;y<ay;y++) FBrenderPutPixel(((int)round((y-q)/m)),y,color);
		} else {
			int x;
			if(bx>ax) for(x=ax;x<bx;x++) FBrenderPutPixel(x,((int)round(m*x+q)),color);
			else for(x=bx;x<ax;x++) FBrenderPutPixel(x,((int)round(m*x+q)),color);
		}
	} else { //vertical line
		int y;
		if(by>ay) for(y=ay;y<by;y++) FBrenderPutPixel(ax,y,color);
		else for(y=by;y<ay;y++) FBrenderPutPixel(bx,y,color);
	}
}

void FillRect(int ulx, int uly, int drx, int dry, unsigned short color) {
	int y,lenght=drx-ulx;
	for(y=uly;y<=dry;y++) DrawHorizontalLine(ulx,y,lenght,color);
}

void DrawButton(int x, int y, bool active, const char *label, ...) {
	FillRect(x,y,x+180,y+30,active?config.colorSchema.buttonEnabled:config.colorSchema.buttonDisabled);
	FBrenderBlitText(x+10,y+10,active?config.colorSchema.buttonLabelEnabled:config.colorSchema.buttonLabelDisabled,active?config.colorSchema.buttonEnabled:config.colorSchema.buttonDisabled,!active,label);
}

//TODO: to be implemented and tested...
/*void FillQuadrangle(int ax, int ay, int bx, int by, int cx, int cy, int dx, int dy, unsigned short color) {
	int coord[4][2]={{ax,ay},{bx,by},{cx,cy},{dx,dy}};
	int n[2]={0,screen.height};
	int s[2]={0,0};
	int w[2]={screen.width,0};
	int e[2]={0,0};
	int i;
	for(i=0;i<4;i++) {
		if(coord[i][1]<n[1]) {n[0]=coord[i][0]; n[1]=coord[i][1];}
		if(coord[i][1]>s[1]) {s[0]=coord[i][0]; s[1]=coord[i][1];}
		if(coord[i][0]<w[0]) {w[0]=coord[i][0]; w[1]=coord[i][1];}
		if(coord[i][0]>e[0]) {e[0]=coord[i][0]; e[1]=coord[i][1];}
	}


	if(w[0]!=n[0] && n[0]!=e[0] && w[0]!=s[0] && s[0]!=e[0]) {
		double wnm=(double)(n[1]-w[1])/(double)(n[0]-w[0]);
		double wnq=w[1]-wnm*w[0];
		double nem=(double)(e[1]-n[1])/(double)(e[0]-n[0]);
		double neq=n[1]-nem*n[0];
		double wsm=(double)(s[1]-w[1])/(double)(s[0]-w[0]);
		double wsq=w[1]-wsm*w[0];
		double sem=(double)(e[1]-s[1])/(double)(e[0]-s[0]);
		double seq=s[1]-sem*s[0];
		int j;
		if(n[0]>s[0]) {
			for(i=w[0];i<s[0];i++) {
				int start=(int)round(wnm*i+wnq);
				int end=(int)round(wsm*i+wsq);
				for(j=start;j<end;j++) FBrenderPutPixel(i,j,color);
			}
			for(i=s[0];i<n[0];i++) {
				int start=(int)round(wnm*i+wnq);
				int end=(int)round(sem*i+seq);
				for(j=start;j<end;j++) FBrenderPutPixel(i,j,color);
			}
			for(i=n[0];i<e[0];i++) {
				int start=(int)round(nem*i+neq);
				int end=(int)round(sem*i+seq);
				for(j=start;j<end;j++) FBrenderPutPixel(i,j,color);
			}
		} else {
			for(i=w[0];i<n[0];i++) {
				int start=(int)round(wnm*i+wnq);
				int end=(int)round(wsm*i+wsq);
				for(j=start;j<end;j++) FBrenderPutPixel(i,j,color);
			}
			for(i=n[0];i<s[0];i++) {
				int start=(int)round(nem*i+neq);
				int end=(int)round(wsm*i+wsq);
				for(j=start;j<end;j++) FBrenderPutPixel(i,j,color);
			}
			for(i=s[0];i<e[0];i++) {
				int start=(int)round(nem*i+neq);
				int end=(int)round(sem*i+seq);
				for(j=start;j<end;j++) FBrenderPutPixel(i,j,color);
			}
		}
	} else {
		//in this case we have to verify if it is a normal rectangle
	}
}*/

void PrintPosition(int latD, int latM, double latS, short N, int lonD, int lonM, double lonS, short E) {
	if(screen.height!=240) {
		FBrenderBlitText(screen.height+28,2,config.colorSchema.text,config.colorSchema.background,false,"%3d %2d' %6.3f\" %c ",latD,latM,latS,N?'N':'S');
		FBrenderBlitText(screen.height+28,12,config.colorSchema.text,config.colorSchema.background,false,"%3d %2d' %6.3f\" %c ",lonD,lonM,lonS,E?'E':'W');
	}
}

void PrintSpeed(double speedKmh, double speedKnots) {
	if(screen.height!=240) {
		switch(config.speedUnit) {
			case KNOTS:
				FBrenderBlitText(screen.height+28,32,config.colorSchema.text,config.colorSchema.background,false,"GS: %7.2f Knots",speedKnots);
				break;
			case MPH:
				FBrenderBlitText(screen.height+28,32,config.colorSchema.text,config.colorSchema.background,false,"GS: %7.2f MPH  ",Km2Miles(speedKmh));
				break;
			case KMH:
			default:
				FBrenderBlitText(screen.height+28,32,config.colorSchema.text,config.colorSchema.background,false,"GS: %7.2f Km/h ",speedKmh);
				/* no break */
		}
	} else switch(config.speedUnit) {
		case KNOTS:
			FBrenderBlitText(screen.height+28,32,config.colorSchema.text,config.colorSchema.background,false,"%.0f Knots",speedKnots);
			break;
		case MPH:
			FBrenderBlitText(screen.height+28,32,config.colorSchema.text,config.colorSchema.background,false,"%.0f MPH  ",Km2Miles(speedKmh));
			break;
		case KMH:
		default:
			FBrenderBlitText(screen.height+28,32,config.colorSchema.text,config.colorSchema.background,false,"%.0f Km/h ",speedKmh);
			/* no break */
	}
}

void PrintNavStatus(int navStatus, char *WPname) {
	char *statusName;
	switch((enum navigatorStatus)navStatus) {
			case NAV_STATUS_NOT_INIT:         statusName=strdup("Nav not set   "); break;
			case NAV_STATUS_NO_ROUTE_SET:     statusName=strdup("No route set  "); break;
			case NAV_STATUS_TO_START_NAV:     statusName=strdup("Ready to start"); break;
			case NAV_STATUS_WAIT_FIX:         statusName=strdup("Waiting FIX   "); break;
			case NAV_STATUS_NAV_TO_DPT:       statusName=strdup("Nav to Depart."); break;
			case NAV_STATUS_NAV_TO_WPT:       statusName=strdup("Nav to WPT    "); break;
			case NAV_STATUS_NAV_TO_DST:
			case NAV_STATUS_NAV_TO_SINGLE_WP: statusName=strdup("Nav to Dest.  "); break;
			case NAV_STATUS_END_NAV:          statusName=strdup("Nav ended     "); break;
			case NAV_STATUS_NAV_BUSY:         statusName=strdup("Busy, planning"); break;
			default:                          statusName=strdup("Unknown       "); break;
	}
	if(screen.height!=240) {
		FBrenderBlitText(screen.height+28,52,config.colorSchema.text,config.colorSchema.background,false,"NAV: %s       ",statusName);
		FBrenderBlitText(screen.height+28,62,config.colorSchema.text,config.colorSchema.background,false,"WPT: %s                 ",WPname);
	} else {
		FBrenderBlitText(screen.height+28,52,config.colorSchema.text,config.colorSchema.background,false,"%s       ",statusName);
		FBrenderBlitText(screen.height+28,62,config.colorSchema.text,config.colorSchema.background,false,"%s               ",WPname);
	}
	free(statusName);
}

void PrintNavRemainingDistWP(double distKm, double averageSpeedKmh, double hours) {
	if(screen.height!=240) {
		switch(config.distUnit) { //Distance To Go
			case NM:
				FBrenderBlitText(screen.height+28,72,config.colorSchema.text,config.colorSchema.background,false,"DTG: %7.3f NM    ",Km2Nm(distKm));
				break;
			case MI:
				FBrenderBlitText(screen.height+28,72,config.colorSchema.text,config.colorSchema.background,false,"DTG: %7.3f Mi    ",Km2Miles(distKm));
				break;
			case KM:
			default:
				FBrenderBlitText(screen.height+28,72,config.colorSchema.text,config.colorSchema.background,false,"DTG: %7.3f Km    ",distKm);
				/* no break */
		}
		if(averageSpeedKmh>0) switch(config.speedUnit) {
			case KNOTS:
				FBrenderBlitText(screen.height+28,92,config.colorSchema.text,config.colorSchema.background,false,"AS: %5.1f Knots   ",Km2Nm(averageSpeedKmh));
				break;
			case MPH:
				FBrenderBlitText(screen.height+28,92,config.colorSchema.text,config.colorSchema.background,false,"AS: %5.1f MPH     ",Km2Miles(averageSpeedKmh));
				break;
			case KMH:
			default:
				FBrenderBlitText(screen.height+28,92,config.colorSchema.text,config.colorSchema.background,false,"AS: %5.1f Km/h    ",averageSpeedKmh);
				/* no break */
		}
	} else switch(config.distUnit) { //Distance To Go
			case NM:
				FBrenderBlitText(screen.height+28,72,config.colorSchema.text,config.colorSchema.background,false,"%.2f NM    ",Km2Nm(distKm));
				break;
			case MI:
				FBrenderBlitText(screen.height+28,72,config.colorSchema.text,config.colorSchema.background,false,"%.2f Mi    ",Km2Miles(distKm));
				break;
			case KM:
			default:
				FBrenderBlitText(screen.height+28,72,config.colorSchema.text,config.colorSchema.background,false,"%.2f Km    ",distKm);
				/* no break */
	}
	if(hours<100 && hours>0) {
		int hour,min;
		float sec;
		convertDecimal2DegMinSec(hours,&hour,&min,&sec); //Estimated Time Enroute
		if(screen.height!=240) FBrenderBlitText(screen.height+28,102,config.colorSchema.text,config.colorSchema.background,0,"ETE: %2d:%02d:%02.0f    ",hour,min,sec);
		else FBrenderBlitText(screen.height+28,102,config.colorSchema.text,config.colorSchema.background,0,"%2d:%02d    ",hour,min);
	} else if(screen.height!=240) FBrenderBlitText(screen.height+28,102,config.colorSchema.text,config.colorSchema.background,0,"ETE: --:--:--    ");
		else FBrenderBlitText(screen.height+28,102,config.colorSchema.text,config.colorSchema.background,0,"--:--    ");
}

void PrintNavDTG(double distRad) { //Distance To Go
	switch(config.distUnit) {
		case NM:
			FBrenderBlitText(screen.height+28,72,config.colorSchema.text,config.colorSchema.background,false,"DTG: %7.3f NM    ",Rad2Nm(distRad));
			break;
		case MI:
			FBrenderBlitText(screen.height+28,72,config.colorSchema.text,config.colorSchema.background,false,"DTG: %7.3f Mi    ",Rad2Mi(distRad));
			break;
		case KM:
		default:
			FBrenderBlitText(screen.height+28,72,config.colorSchema.text,config.colorSchema.background,false,"DTG: %7.3f Km    ",Rad2Km(distRad));
			/* no break */
	}
}

void PrintNavTrackATD(double atdRad) {
	if(screen.height!=240) switch(config.distUnit) {
		case NM:
			FBrenderBlitText(screen.height+28,82,config.colorSchema.text,config.colorSchema.background,false,"ATD: %7.3f NM  ",Rad2Nm(atdRad));
			break;
		case MI:
			FBrenderBlitText(screen.height+28,82,config.colorSchema.text,config.colorSchema.background,false,"ATD: %7.3f Mi  ",Rad2Mi(atdRad));
			break;
		case KM:
		default:
			FBrenderBlitText(screen.height+28,82,config.colorSchema.text,config.colorSchema.background,false,"ATD: %7.3f Km  ",Rad2Km(atdRad));
			/* no break */
	}
}

void PrintAltitude(double altMt, double altFt) {
	FBrenderBlitText(screen.height+28,133,config.colorSchema.text,config.colorSchema.background,false,"%.0f Ft   %.0f m    ",altFt,altMt);
}

void PrintVerticalSpeed(double FtMin) {
	//FBrenderBlitText(2,42,config.colorSchema.text,config.colorSchema.background,str,false,"VS: %.0f Ft/min   ",FtMin);
}

void PrintTurnRate(double DegMin) {
	//FBrenderBlitText(2,52,config.colorSchema.text,config.colorSchema.background,str,false,"TR: %.2f Deg/min  ",DegMin);
}

void PrintNavRemainingDistDST(double distKm, double averageSpeedKmh, double timeHours) {
	if(screen.height!=240) {
		switch(config.distUnit) {
			case NM:
				FBrenderBlitText(screen.height+28,172,config.colorSchema.text,config.colorSchema.background,false,"Tot DTG: %7.3f NM     ",Km2Nm(distKm));
				break;
			case MI:
				FBrenderBlitText(screen.height+28,172,config.colorSchema.text,config.colorSchema.background,false,"Tot DTG: %7.3f Mi     ",Km2Miles(distKm));
				break;
			case KM:
			default:
				FBrenderBlitText(screen.height+28,172,config.colorSchema.text,config.colorSchema.background,false,"Tot DTG: %7.3f Km     ",distKm);
				/* no break */
		}
		if(averageSpeedKmh>0) switch(config.speedUnit) {
			case KNOTS:
				FBrenderBlitText(screen.height+28,182,config.colorSchema.text,config.colorSchema.background,false,"AS: %5.1f Knots   ",Km2Nm(averageSpeedKmh));
				break;
			case MPH:
				FBrenderBlitText(screen.height+28,182,config.colorSchema.text,config.colorSchema.background,false,"AS: %5.1f MPH     ",Km2Miles(averageSpeedKmh));
				break;
			case KMH:
			default:
				FBrenderBlitText(screen.height+28,182,config.colorSchema.text,config.colorSchema.background,false,"AS: %5.1f Km/h    ",averageSpeedKmh);
				/* no break */
		}
	} else switch(config.distUnit) {
		case NM:
			FBrenderBlitText(screen.height+28,172,config.colorSchema.text,config.colorSchema.background,false,"%.2f NM     ",Km2Nm(distKm));
			break;
		case MI:
			FBrenderBlitText(screen.height+28,172,config.colorSchema.text,config.colorSchema.background,false,"%.2f Mi     ",Km2Miles(distKm));
			break;
		case KM:
		default:
			FBrenderBlitText(screen.height+28,172,config.colorSchema.text,config.colorSchema.background,false,"%.2f Km     ",distKm);
			/* no break */
	}
	//TODO: Continue here with the conversion to smaller screen
	if(timeHours<48 && timeHours>0) {
		if(timeHours>24) timeHours-=24;
		int hour,min;
		float sec;
		convertDecimal2DegMinSec(timeHours,&hour,&min,&sec);
		FBrenderBlitText(screen.height+28,192,config.colorSchema.text,config.colorSchema.background,0,"ETA: %2d:%02d:%02.0f    ",hour,min,sec);
	} else FBrenderBlitText(screen.height+28,192,config.colorSchema.text,config.colorSchema.background,0,"ETA: --:--:--    "); //Estimated Time of Arrival
}

void PrintTime(int hour, int minute, float second, short waring) {
	FBrenderBlitText(screen.height+28,240,waring?config.colorSchema.warning:config.colorSchema.ok,config.colorSchema.background,false,"UTC: %02d:%02d:%02.0f",hour,minute,second);
}

void PrintDate(int day, int month, int year) {
	//FBrenderBlitText(2,230,config.colorSchema.text,config.colorSchema.background,0,"Date: %2d/%02d/%4d",day,month,year);
}

void PrintNumOfSats(int activeSats, int satsInView) {
	unsigned short color;
	if(activeSats>3) color=config.colorSchema.ok; //green
	else if(activeSats==3) color=config.colorSchema.warning; //yellow
	else color=config.colorSchema.caution; //red
	FBrenderBlitText(screen.height+28,250,color,config.colorSchema.background,false,"SAT: %2d/%2d",activeSats,satsInView);
}

void PrintFixMode(int fixMode) {
	switch((enum GPSmode)fixMode) {
		case MODE_GPS_FIX: FBrenderBlitText(screen.height+28,260,config.colorSchema.warning,config.colorSchema.background,false,"FIX: GPS Fix"); break;
		case MODE_3D_FIX: FBrenderBlitText(screen.height+28,260,config.colorSchema.ok,config.colorSchema.background,false,"FIX: 3D mode"); break;
		case MODE_2D_FIX: FBrenderBlitText(screen.height+28,260,config.colorSchema.warning,config.colorSchema.background,false,"FIX: 2D mode"); break;
		case MODE_NO_FIX: FBrenderBlitText(screen.height+28,260,config.colorSchema.caution,config.colorSchema.background,false,"FIX: No Fix "); break;
		default: FBrenderBlitText(screen.height+28,260,config.colorSchema.caution,config.colorSchema.background,false,"FIX: Unknown"); break;
	}
}

void PrintDiluitions(float pDiluition, float hDiluition, float vDiluition) {
	//FBrenderBlitText(2,260,config.colorSchema.text,config.colorSchema.background,0,"DOP P:%4.1f H:%4.1f V:%4.1f",pDiluition,hDiluition,vDiluition);
}
