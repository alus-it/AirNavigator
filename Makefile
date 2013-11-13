# ============================================================================
# Name : Makefile
# Since : 22/4/2011
# Author : Alberto Realis-Luc <alberto.realisluc@gmail.com>
# Web : http://www.alus.it/airnavigator/
# Copyright : (C) 2010-2013 Alberto Realis-Luc
# License : GNU GPL v2
# Repository : https://github.com/AirNavigator/AirNavigator.git
# Last change : 13/11/2013
# Description : Makefile of AirNavigator for TomTom devices
# ============================================================================

# Usual path of the toolchain: /usr/local/cross/gcc-3.3.4_glibc-2.3.2/bin/ (must be added to your path)

# AirNavigator version string
VERSION = 0.2.8

# Compiler and tools
CC = arm-linux-gcc
STRIP = arm-linux-strip

# Compiler and linker options
CFLAGS = -c -O3 -fPIC -Wall
LFLAGS = -lm -lpthread

# Source and binary paths
SRC = src/
BIN = bin/

# Libs sources and libs headers path
LIBSRC = libs/

# Headers for others external libs path (TomTom stuff)
INC = include/

# Source and object files lists
CFILES = main.c FBrender.c TSreader.c AirCalc.c BlackBox.c HSI.c Navigator.c NMEAreader.c Configuration.c Geoidal.c
OBJS = $(patsubst %.c, $(BIN)%.o, $(CFILES))

# Final distribution destination folder
DIST = release/

# Libraries destination path
LIB = $(DIST)AirNavigator/lib/

# List of libraries
LIBFILES = libroxml.so
LIBS= $(patsubst %.so, $(LIB)%.so, $(LIBFILES))

# Name of the ditributions zip files
ZIPNAME = AirNavigator_$(subst .,-,$(VERSION))
ZIPSTANDNAME = AirNavigatorStandalone_$(subst .,-,$(VERSION))

# Path of the ttsystem image file
SYSTEMIMAGE = utility/standaloneImage/


### Build dependencies
all: $(DIST)AirNavigator/AirNavigator

$(DIST)AirNavigator/AirNavigator: $(OBJS) $(LIBS)
	$(CC) $(LFLAGS) $(OBJS) -L$(LIB) -lroxml -o $@
	$(STRIP) $@

# Create bin directory if missing
$(OBJS): | $(BIN)
$(BIN):
	mkdir -p $(BIN)

# Create lib directory if missing
$(LIBS): | $(LIB) 
$(LIB):
	mkdir -p $(LIB)

$(BIN)main.o: $(SRC)main.c $(SRC)AirNavigator.h $(SRC)Configuration.h $(SRC)FBrender.h $(SRC)TSreader.h $(SRC)NMEAreader.h $(SRC)Navigator.h $(SRC)AirCalc.h $(SRC)BlackBox.h $(SRC)HSI.h $(SRC)Geoidal.h
	$(CC) $(CFLAGS) -D'VERSION="$(VERSION)"' -I $(INC) $< -o $@

$(BIN)NMEAreader.o: $(SRC)NMEAreader.c $(SRC)NMEAreader.h $(SRC)AirNavigator.h $(SRC)Configuration.h $(SRC)AirCalc.h $(SRC)Geoidal.h $(SRC)FBrender.h $(SRC)HSI.h $(SRC)Navigator.h $(SRC)BlackBox.h
	$(CC) $(CFLAGS) $< -o $@

$(BIN)SiRFreader.o: $(SRC)SiRFreader.c $(SRC)SiRFreader.h $(SRC)AirNavigator.h
	$(CC) $(CFLAGS) $< -o $@

$(BIN)Navigator.o: $(SRC)Navigator.c $(SRC)Navigator.h $(SRC)Configuration.h $(SRC)AirCalc.h $(SRC)NMEAreader.h $(SRC)FBrender.h $(SRC)HSI.h $(SRC)AirNavigator.h $(LIBSRC)libroxml/roxml.h
	$(CC) $(CFLAGS) -I $(LIBSRC) $< -o $@

$(BIN)HSI.o: $(SRC)HSI.c $(SRC)HSI.h $(SRC)FBrender.h $(SRC)AirCalc.h $(SRC)Configuration.h
	$(CC) $(CFLAGS) $< -o $@

$(BIN)BlackBox.o: $(SRC)BlackBox.c $(SRC)BlackBox.h $(SRC)Configuration.h $(SRC)AirCalc.h
	$(CC) $(CFLAGS) $< -o $@

$(BIN)FBrender.o: $(SRC)FBrender.c $(SRC)FBrender.h $(SRC)Navigator.h $(SRC)AirCalc.h $(SRC)NMEAreader.h $(SRC)Configuration.h
	$(CC) $(CFLAGS) -DLINUX_TARGET -I $(INC) $< -o $@

$(BIN)Configuration.o: $(SRC)Configuration.c $(SRC)Configuration.h $(SRC)AirNavigator.h $(SRC)FBrender.h $(LIBSRC)libroxml/roxml.h
	$(CC) $(CFLAGS) -I $(LIBSRC) $< -o $@

$(BIN)Geoidal.o: $(SRC)Geoidal.c $(SRC)Geoidal.h $(SRC)AirNavigator.h
	$(CC) $(CFLAGS) $< -o $@

$(BIN)TSreader.o: $(SRC)TSreader.c $(SRC)TSreader.h
	$(CC) $(CFLAGS) -I $(INC) $< -o $@

$(BIN)AirCalc.o: $(SRC)AirCalc.c $(SRC)AirCalc.h
	$(CC) $(CFLAGS) $< -o $@


### Lib dependencies
$(LIB)libroxml.so: $(LIBSRC)libroxml/Makefile
	make -C $(LIBSRC)libroxml
	cp -f $(LIBSRC)libroxml/libroxml.so $@


### Clean dependencies
clean: objclean libclean

# To clean just objects, executable and zip file
objclean:
	@rm -f $(BIN)*
	@rm -f $(DIST)AirNavigator/AirNavigator
	@rm -f $(DIST)$(ZIPNAME).zip

# To clean just the libraries
libclean:
	make -C $(LIBSRC)libroxml clean
	@rm -f $(LIB)*

	
### Make the zip files with the final distribution normal and standalone
zip: all $(SYSTEMIMAGE)ttsystem $(SYSTEMIMAGE)config.xml
	@rm -f $(DIST)*.zip
	mkdir -p $(DIST)AirNavigator/Tracks
	cd $(DIST); zip -9 -T -x "*.git*" "*.svn*" "*CVS*" "*Thumbs.db*" -r $(ZIPNAME).zip . ; cd ..;
	cp $(SYSTEMIMAGE)ttsystem $(DIST)
	mv $(DIST)AirNavigator/config.xml $(DIST)
	cp $(SYSTEMIMAGE)config.xml $(DIST)AirNavigator/
	cd $(DIST); zip -9 -T -x "*.git*" "*.svn*" "*CVS*" "*Thumbs.db*" -r $(ZIPSTANDNAME).zip ttsystem AirNavigator/* README.txt ; cd ..;
	rm $(DIST)ttsystem
	mv -f $(DIST)config.xml $(DIST)AirNavigator/

