#!/bin/sh

#Entering AirNavigator directory installation
AIRNAV_DIR=`echo $0 | sed 's/AirNavigator.sh//'` 
cd $AIRNAV_DIR

#To be sure to get absolute path
AIRNAV_DIR=`pwd`

#Set library path
export LD_LIBRARY_PATH=${AIRNAV_DIR}/lib

#Start AirNavigator
./AirNavigator

