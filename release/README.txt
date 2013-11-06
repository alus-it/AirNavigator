AirNavigator - README file

The aim of this project is to develop an experimental GPS navigation application for ultralight/microlight aircrafts, using as hardware a vehicular and inexpensive TomTom navigator. The program, given a flight plan defined by the user, is able to provide navigation indications, including an HSI (Horizontal Situation Indicator), along all the way points of the route until the final destination.


DISCLAIMER

This software comes without any warranty. This is an experimental project, actually the main purpose it is just to understand if it works and to test it. During a flight this program must not be used as the only navigation system.


REQUIREMENTS

To use this software you need and old TomTom device with a screen resolution of 480x272 pixel. At the moment the program is developed specifically for that resolution. The program has been tested only on the TomTom One XL. If ran with a different screen resolution the program may display the data in a not readable way. This is the list of all the TomTom devices with a 480x272 resolution:

* Series GO: 510, 520, 530, 540, 550
* GO 630
* Series GO: 710, 720, 730, 740, 750
* Series GO: 910, 920, 930, 940, 950
* Series One: XL, XL S, XL HDT
* XL

On all the devices listed above the program should work. Anyway I repeat: I tested it only on the "One XL" model and I still had any notice about the program working correctly or not on different models.
Anyway, soon, I'm going to adapt the program to work also with the resolution of 320x240 used by the others TomTom devices. In this way the HSI will work almost all TomTom navigators, including the basic models like the "One" or the well known "GO 300".

ASSUMPTIONS

We expect that the user planned the flight before, on the ground, studying the chart and producing a flight plan as GPX file that will be loaded on the navigator.


HOW IT WORKS

Once started, the program will ask the user to select a flight plan then it will process data from the integrated GPS receiver. The HSI will give visual indications to the next way point until the final destination is reached.


INSTALLATION

Which one?
It is possible to install this software in two ways: standalone or not.
The AirNavigator executable is exactly the same in both cases; the difference is that in the standalone distribution the software is capable to run completely independently by the TomTom software using his own operating system.
Otherwise with the normal distribution, after the installation you will have a new icon in the menu, to start the program, so in this way you can continue to use your TomTom also for his original purpose, without the need to switch 2 different SDs.
Unfortunately not all the TomTom software versions allow to run another program easily with an icon in the menu, in this case I suggest to use AirNavigator as standalone on another separate SD card.

In any case, before the installation I warmly recommend to make a full backup of all the content of the memory of your TomTom.

Standalone distribution
In this case you just need an empty SD memory card (or the internal memory of your TomTom) where to extract all the contents of the standalone distribution zip file.
In this way you can run AirNavigator without having problems with the TomTom software blocked because, for example, you don't have any map installed.

Normal distribution
To install AirNavigator on your TomTom device follow these steps:
1. Download AirNavigator's normal distribution zip file
2. Connect your TomTom to the PC using the USB cable or insert the TomTom's SD card in your PC
3. Extract the folders "AirNavigator" and "SdkRegistry" from the zip file into the root of TomTom's memory or SD card
4. Unmount (safely remove) the TomTom device (or SD card) and restart it
5. In the main menu you should see at the end an icon to start AirNavigator


UNINSTALLATION

Normal distribution
1. Before uninstall, you may would like to make a backup of your routes and tracks from Routes and Tracks folders inside the AirNavigator directory
2. Form the memory of the TomTom device delete the "AirNavigator" folder and all its contents
3. Inside the folder "SdkRegistry" delete the two files: "AirNavigator.cap" and "AirNavigator.bmp"


UPGRADE

If you wish to upgrade to a new version of AirNavigator simply uninstall it and then install the new version.


DISPLAYED DATA

On the left side, Horizontal Situation Indicator:

	In white the compass rose
	In red the indicator of current true heading
	In green the course indicator to the next way point
	In yellow the course deviation indicator, displayed also in meters
    
In the center, the altitude scale:
	
	Every marker represents 50 Ft
	The numbers on the scale represent hundreds of Ft
	The white (red if out of scale) arrow on the left points the altitude we should keep to respect the route
	The green arrow on the right (center of the scale) points to the actual altitude in Ft respect m.s.l.

Right side:

	Latitude expressed in degrees, minutes and seconds
	Longitude expressed in degrees, minutes and seconds
    
	GS: Ground Speed
    
	NAV: Current status of the navigator
	WPT: Name of the next way point
	DTG: Distance To Go, remaining distance to the next way point
	ATD: Along Track Distance, distance covered on the track
	AS: Average Speed on the current track
	ETE: Estimated Time En route, time it will take to the next way point

	ALT: Altitude in meters and feet from mean sea level

	Tot DTG: Total DTG, total remaining distance along all the route
	AS: Average Speed measured along all the covered route
	ETA: Estimated Time of Arrival, expressed as UTC time

	UTC: UTC time received from the GPS
	SAT: Active GPS satellites / GPS satellites in view
	FIX: Type of GPS fix


FEATURES

	Tracking of: position, altitude, speed and direction during flight in GPX track files
	Calculation of routes using Great Circle distance
	Calculation of sunrise and sunset time along the route
	Estimation of remaining fuel
	XML configuration file


ROUTES AND TRACKS AS GPX FILES

GPX is an open XML schema that allow to store in a file collections of: way points, routes and tracks. It is always more used by many GPS devices and applications, because this format is independent from any GPS device producer it is often used to exchange data between devices and software's of different brands. For example: also with the popular Google Earth it is possible to open GPX files and see routes and tracks even in 3D view. More information about the GPX format can be found on its official website: http://www.topografix.com/gpx.asp


AirNavigator and the GPX format

Once started AirNavigator looks for routes inside the GPX files loaded by the user in the folder: /AirNavigator/Routes/. So in order to allow AirNavigator to fly your routes you should copy your GPX flight plans a in the directory: /AirNavigator/Routes/ of the memory of your TomTom navigator.


Preparing a GPX route

There are several ways to do that, if your flight planning software supports the GPX format then, you don't need to read further, just save your route as GPX file. If this is not possible there are many software's able to convert between different formats. A good and free converter is for example gpsbabel. Let's say that you planned a flight with OziExplorer so you have your route saved as: FlightPlan.rte in the OziExplorer format. In this case using gpsbabel you can convert your route to GPX with this command:

	gpsbabel -i ozi -f FlightPlan.rte -o gpx -F FlightPlan.gpx

Or if you have the route in the Google Earth format as FlightPlan.kml:

	gpsbabel -i kml -f FlightPlan.kml -o gpx -F FlightPlan.gpx

Another way to prepare your GPX route is to use the free on line tools of FlightUtilities.com you can add way points to your flight plan simply clicking on the map and then save it as GPX file. There is also a Windows version of the same program that allow you to prepare your route off line in a more traditional way, finding your way points on a real chart on paper and giving the coordinates to the software that will produce the GPX flight plan.


GPX tracks

When AirNavigator gets a fix from the GPS receiver starts to record: position, altitude, heading and ground speed in a GPX track file saved in the directory: /AirNavigator/Tracks/. Every track file is named with his date and UTC time of creation. You can download them from your device and analyze the track of your flight using for example Google Earth. To prevent excessive size of track files AirNavigator doesn't record every time a sentence is received, but this is done considering a maximum interval of time and a minimum distance covered from the last tracked point. This means that, for example, if you stand for one hour in the same place without moving AirNavigator will record this position only one time.


USAGE AND TEST PROCEDURE

By downloading and using this program you understand that it just an experiment and must not used for real navigation purposes.
Actually the only one use of AirNavigator is to test itself, here the procedure:

 1. Prepare your flight plan as described previously: a flight that you know where you can recognize if the program is wrong
 2. You can check your GPX flight plans with Google Earth, just to see if you're not going to "fly inside mountains"
 3. Upload your GPX flight plan in the Routes folder of AirNavigator on the TomTom navigator
 4. Start AirNavigator
 5. At the first screen of AirNavigator you can select the desired GPX file with your route, then choose "LOAD"
 6. Now AirNavigator is displaying the main screen, "Wait to get a fix" or "Ready to start" depending if you got already a fix 
 7. When you have a fix, touch the screen once to start the navigation immediately before the take off.
 8. During the the flight, try to understand if, what AirNavigator says, have sense...
 9. When arrived you can touch the screen another time to reverse the route and start to fly back to the original departure
10. After the second touch of the screen, AirNavigator can be closed touching the upper left corner of the screen
11. Download from the Track folder of AirNavigator the GPX track of your flight, again with Google Earth you can display it for analysis purposes


CONFIGURATION FILE

In the "AirNavigator" folder there is the "config.xml" file. In this file there are all the configuration parameters used by the program. If needed you can customize the parameters by opening end editing this file. In this file only the values can be changed the names of the elements and attributes must remain the same otherwise AirNavigator will not find them. Let's explain every element of this file:

Aircraft
<aircraft>
	<!-- measure units: speed: Km/h, fuel consumption: liters/hour, fuel capacity: liters -->
	<speeds cruise="100" Vx="90" Vy="100" Vs0="45" Vs="55" Vfe="86" Va="135" Vno="160" Vne="180" endScale="185" />
	<fuel consumption="15" capacity="70" />
</aircraft>
In the "aircraft" element it is possible to configure the parameters of the plane where you intend to use the navigator: speeds and fuel. The cruise speed will be used as the average speed when there is still not enough data received to calculate the real average speed. Here the speeds must be expressed in Km/h, fuel capacity in liters and the fuel consumption in l/h.

Measure units
<measureUnits>
	<!-- possible values for horizontal speed : Kmh, Knots, MPH  -->
	<!-- possible values for vertical speed : FtMin, ms  -->
	<speeds horizontal="Kmh" vertical="FtMin" />
	<!-- possible values for distance : Km, NM, Mile  -->
	<!-- possible values for track error : m, Ft, NM  -->
	<distances distance="Km" trackError="m" />
</measureUnits>
Here is possible to choose the measure units for the data displayed on the screen for distances ad speeds

Navigator
<navigator>
	<!-- measure units: altitude and distances: meters, angles: degrees -->
	<takeOff diffAlt="50" />
	<navParameters trackErrorTolearnce="5" deptDistTolerance="1000" />
	<sunZenith angle="96" />
</navigator>
* takeOff diffAlt: it is the difference of altitude that once detected together with a speed higher than the stall speed will be considered as the start of the flight.
* trackErrorTolearnce: if the track error (XTD) is smaller than trackErrorTolearnce the navigator will use an easier calculation algorithm in order to be faster we are already in route.
* deptDistTolerance: if we are at distance less than deptDistTolerance from our departure location than AirNavigator will consider us still at the departure and starting the flight, else it will consider us away from the departure and the flight already started to the next way point
* sunZenith angle: it is the sun Zenith angle in degrees used to calculate sunrise and sunset times:
	Official:	90° 50'
	Civil:		96°
	Nautical:	102°
	Astronomical:	108°

Track Recorder
<trackRecorder>
	 <!-- measure units: interval time: seconds, minimum distance: meters -->
	 <update timeInterval="5" distanceInterval="10" />
</trackRecorder>
* distanceInterval: it is the minimum distance that we have to cover from the previous recorded position to allow the recorder to record a new one
* timeInterval: it is the minimum interval of time that the recorder let to elapse between every recorded position

Colors schema configuration
<colorSchema>
	<colors 
	background="0000"
	compassRose="FFFF"
	dirMarker="F000"
	magneticDir="00F0"
	routeIndicator="0F00"
	cdi="FF00"
	cdiScale="FFFF"
	altScale="FFFF"
	vsi="FFFF"
	altMarker="FFFF"
	text="0F00"
	warning="F000" />
</colorSchema>
Here it is possible to choose the color of everything AirNavigator will display on the screen of your TomTom, the colors must be encoded with a four digit hexadecimal number starting from 0000 that is black up to FFFF that is white. (Red=F000, green=0F00, blue=00F0, yellow=FF00)
* background: the color of the background
* compassRose: color of the compass and its scale
* dirMarker: color of the marker on the top of the compass indicating the current heading true direction
* magneticDir: color of the numbers indicating the current magnetic heading (not displayed because from TomTom GPS unit it is equal to the true one)
* routeIndicator: color of the arrow in the HSI indicating the direction of the route
* cdi: color of the course deviation indicator inside the HSI
* cdiScale: color of the course deviation scale
* altScale: color of the altitude/VSI scale
* vsi: color of the VSI marker on the left pointing the route altitude
* altMarker: color of the altitude marker pointing the current altitude
* text: color of the textual informations displayed on the left
* warning: color used to highlight some potential problem: VSI marker out of scale (too high or too low respect the route) or no GPS fix.

GPS receiver configuration
<GPSreceiver devName="/var/run/gpsfeed" baudRate="115200" dataBits="8" stopBits="1" parity="0" />
This is the configuration about how AirNavigator connects to the internal GPS receiver of the device. Those settings should be the same for almost all the TomTom devices.
If you are running AirNavigator in parallel with the TomTom software the device name should be: /var/run/gpsfeed
Otherwise if you are running the application standalone the device name must be: /var/run/gpspipe


VERSION HISTORY

Version 0.0.1
	Implemented the NMEA parser

Version 0.0.2
	Implemented the navigation algorithm

Version 0.0.3
	Implemented the graphics interface to show the HSI

Version 0.0.6
	Added the ability to load a GPX flight plan

Version 0.0.8
	Improved the selection of data from the NMEA protocol

Version 0.1.0
	Various corrections and improvements to the navigation algorithm

Version 0.1.2
	Added the track recorder feature

Version 0.1.4
	Added XML configuration file

Version 0.1.6
	Implemented a smarter way to understand if the next way point is reached

Version 0.1.8
	Added more parameters to configure in the configuration file

Version 0.2.0
	Implemented the VSI bar to show the altitude to keep in route

Version 0.2.1
	Various corrections and improvements, added the possibility to configure the colors

Version 0.2.2
	Corrected a problem in loading the background color

Version 0.2.3
	Libraries are now compiled and loaded apart
	Started the implementation of a SiRF parser (but not yet used in this version)
	Essential menu, displayed at start-up, just to choose the flight plan from the routes folder
	Generation of separated navigation route log files for each route loaded
	Several bugs fixed (including the negative ATD)

Version 0.2.4
	Calculation of altitude delta respect WGS84 geoid with interpolation using EGM96 binary file
	Added the possibility to choose between some different measure units for speeds and distances in the configuration file

Version 0.2.5
	Adaptations and improvements to make it able to run standalone without original TomTom software

Version 0.2.6
	Switch to next Way Point made when crossing bisector lines
	Magnetic heading not displayed because from TomTom's GPS is always equal to true one
	Crash when heading to the final destination fixed
	Create a new track file when reversing the route to return back
	Some other less relevant bug fixed

Version 0.2.7
	TODO...

CONTACTS
Author: Alberto Realis-Luc
Web: http://www.alus.it/airnavigator/
E-mail: airnavigator@alus.it
