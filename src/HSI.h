//============================================================================
// Name        : HSI.h
// Since       : 16/9/2011
// Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
// Web         : http://www.alus.it/airnavigator/
// Copyright   : (C) 2010-2014 Alberto Realis-Luc
// License     : GNU GPL v2
// Repository  : https://github.com/AirNavigator/AirNavigator.git
// Last change : 11/12/2011
// Description : Header of HSI.c to manage the Horizontal Situation Indicator
//============================================================================

#ifndef HSI_H_
#define HSI_H_

#include "Common.h"

void HSIfirstTimeDraw(double direction, double course, double cdiMt, bool onlyDirection);
void HSIdraw(double directionDeg, double courseDeg, double courseDeviationMt, bool force, bool onlyDirection);
void HSIupdateDir(double trueDirectionDeg, double magneticDirectionDeg);
void HSIupdateCDI(double courseDeg, double courseDeviationMt);
void HSIdrawVSIscale(double altFt);
void HSIupdateVSI(double expectedAltFt);

#endif /* HSI_H_ */
