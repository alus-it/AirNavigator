# ============================================================================
# Name : Makefile
# Since : 22/4/2011
# Author : Alberto Realis-Luc <alberto.realisluc@gmail.com>
# Web : http://www.alus.it/airnavigator/
# Copyright : (C) 2010-2013 Alberto Realis-Luc
# License : GNU GPL v2
# Repository : https://github.com/AirNavigator/AirNavigator.git
# Last change : 6/12/2013
# Description : Makefile of AirNavigator for TomTom devices
# ============================================================================


# AirNavigator version string
VERSION = 0.3.0

# Add toolchain location to the path
export PATH := $(PATH):/usr/local/cross/gcc-3.3.4_glibc-2.3.2/bin

# Compiler and tools
CC = arm-linux-gcc
STRIP = arm-linux-strip

# Compiler and linker options
WARN_CFLAGS = -std=gnu99 -pedantic -Wall -Wshadow -Wpointer-arith -Wcast-qual -Wstrict-prototypes -Wmissing-prototypes -Wno-unused-parameter -Werror
CFLAGS = -c -O3 -fPIC -mcpu=arm920t $(WARN_CFLAGS)
LFLAGS = -lm -lpthread

# Source and binary paths
SRC = src/
BIN = bin/

# Libs sources and libs headers path
LIBSRC = libs/

# Headers for others external libs path (TomTom stuff)
INC = include/

# List of C source files
CFILES =            \
	AirCalc.c       \
	BlackBox.c      \
	Common.c        \
	Configuration.c \
	FBrender.c      \
	Geoidal.c       \
	GPSreceiver.c   \
	HSI.c           \
	main.c          \
	Navigator.c     \
	NMEAparser.c    \
	TSreader.c
#	SiRFparser.c    \

# List of object files
OBJS = $(patsubst %.c, $(BIN)%.o, $(CFILES))

# Final distribution destination folder
DIST = release/

# Libraries destination path
LIB = $(DIST)AirNavigator/lib/

# List of libraries
LIBFILES = libroxml.so
LIBS= $(patsubst %.so, $(LIB)%.so, $(LIBFILES))

# Path of mounted TomTom device disk
DEVICE = /media/INTERNAL/

# Name of the ditributions zip files
ZIPNAME = AirNavigator_$(subst .,-,$(VERSION))
ZIPSTANDNAME = AirNavigatorStandalone_$(subst .,-,$(VERSION))

# Path of the ttsystem image file
SYSTEMIMAGE = utility/standaloneImage/


### Build dependencies
all: $(DIST)AirNavigator/AirNavigator

$(DIST)AirNavigator/AirNavigator: $(OBJS) $(LIBS)
	@echo Linking all into: $@
	@$(CC) $(LFLAGS) $(OBJS) -L$(LIB) -lroxml -o $@
	@$(STRIP) $@

# Create bin directory if missing
$(OBJS): | $(BIN)
$(BIN):
	mkdir -p $(BIN)

# Create lib directory if missing
$(LIBS): | $(LIB) 
$(LIB):
	mkdir -p $(LIB)

$(BIN)main.o: $(SRC)main.c $(SRC)Common.h $(SRC)Configuration.h $(SRC)FBrender.h $(SRC)TSreader.h $(SRC)GPSreceiver.h $(SRC)Navigator.h $(SRC)AirCalc.h $(SRC)BlackBox.h $(SRC)HSI.h $(SRC)Geoidal.h
	@echo Compiling: $<
	@$(CC) $(CFLAGS) -D'VERSION="$(VERSION)"' -I $(INC) $< -o $@

$(BIN)GPSreceiver.o: $(SRC)GPSreceiver.c $(SRC)GPSreceiver.h $(SRC)NMEAparser.h $(SRC)SiRFparser.h $(SRC)Common.h $(SRC)Configuration.h $(SRC)AirCalc.h $(SRC)Geoidal.h $(SRC)FBrender.h $(SRC)HSI.h $(SRC)BlackBox.h
	@echo Compiling: $<
	@$(CC) $(CFLAGS) -I $(INC) $< -o $@

$(BIN)NMEAparser.o: $(SRC)NMEAparser.c $(SRC)NMEAparser.h $(SRC)GPSreceiver.h $(SRC)Common.h $(SRC)AirCalc.h $(SRC)Geoidal.h $(SRC)FBrender.h $(SRC)HSI.h $(SRC)Navigator.h $(SRC)BlackBox.h
	@echo Compiling: $<
	@$(CC) $(CFLAGS) $< -o $@

$(BIN)SiRFparser.o: $(SRC)SiRFparser.c $(SRC)SiRFparser.h $(SRC)GPSreceiver.h $(SRC)Common.h
	@echo Compiling: $<
	@$(CC) $(CFLAGS) $< -o $@

$(BIN)Navigator.o: $(SRC)Navigator.c $(SRC)Navigator.h $(SRC)Configuration.h $(SRC)AirCalc.h $(SRC)GPSreceiver.h $(SRC)FBrender.h $(SRC)HSI.h $(SRC)Common.h $(LIBSRC)libroxml/roxml.h
	@echo Compiling: $<
	@$(CC) $(CFLAGS) -I $(LIBSRC) $< -o $@

$(BIN)HSI.o: $(SRC)HSI.c $(SRC)HSI.h $(SRC)FBrender.h $(SRC)AirCalc.h $(SRC)Configuration.h
	@echo Compiling: $<
	@$(CC) $(CFLAGS) $< -o $@

$(BIN)BlackBox.o: $(SRC)BlackBox.c $(SRC)BlackBox.h $(SRC)Common.h $(SRC)Configuration.h $(SRC)AirCalc.h
	@echo Compiling: $<
	@$(CC) $(CFLAGS) $< -o $@

$(BIN)FBrender.o: $(SRC)FBrender.c $(SRC)FBrender.h $(SRC)Navigator.h $(SRC)AirCalc.h $(SRC)GPSreceiver.h $(SRC)Configuration.h
	@echo Compiling: $<
	@$(CC) $(CFLAGS) -DLINUX_TARGET -I $(INC) $< -o $@

$(BIN)Configuration.o: $(SRC)Configuration.c $(SRC)Configuration.h $(SRC)Common.h $(SRC)FBrender.h $(LIBSRC)libroxml/roxml.h
	@echo Compiling: $<
	@$(CC) $(CFLAGS) -I $(LIBSRC) $< -o $@

$(BIN)Geoidal.o: $(SRC)Geoidal.c $(SRC)Geoidal.h $(SRC)Common.h
	@echo Compiling: $<
	@$(CC) $(CFLAGS) $< -o $@

$(BIN)TSreader.o: $(SRC)TSreader.c $(SRC)TSreader.h $(SRC)Common.h
	@echo Compiling: $<
	@$(CC) $(CFLAGS) -I $(INC) $< -o $@

$(BIN)AirCalc.o: $(SRC)AirCalc.c $(SRC)AirCalc.h $(SRC)Common.h
	@echo Compiling: $<
	@$(CC) $(CFLAGS) $< -o $@

$(BIN)Common.o: $(SRC)Common.c $(SRC)Common.h
	@echo Compiling: $<
	@$(CC) $(CFLAGS) $< -o $@


### Lib dependencies
$(LIB)libroxml.so: $(LIBSRC)libroxml/Makefile
	@echo Building library: $@
	@make -C $(LIBSRC)libroxml
	@cp -f $(LIBSRC)libroxml/libroxml.so $@


### Clean dependencies
clean: objclean libclean

# To clean just objects, executable and zip file
objclean:
	@echo Cleaning: objects, executable and distribution files
	@rm -f $(BIN)*
	@rm -f $(DIST)AirNavigator/AirNavigator
	@rm -f $(DIST)*.zip

# To clean just the libraries
libclean:
	@echo Cleaning: libraries and libraries object files
	@make -C $(LIBSRC)libroxml clean
	@rm -f $(LIB)*


### Upload just the executable on the TomTom
upload: all
	@echo Uploading new executable on the TomTom
	@cp -f $(DIST)AirNavigator/AirNavigator $(DEVICE)AirNavigator/


### Make the zip files with the final distribution normal and standalone
zip: all $(SYSTEMIMAGE)ttsystem $(SYSTEMIMAGE)config.xml
	@echo Creating distributions zip files
	@rm -f $(DIST)*.zip
	@mkdir -p $(DIST)AirNavigator/Tracks
	cd $(DIST); zip -9 -T -x "*.git*" "*.svn*" -r $(ZIPNAME).zip . ; cd ..;
	@cp $(SYSTEMIMAGE)ttsystem $(DIST)
	@mv $(DIST)AirNavigator/config.xml $(DIST)
	@cp $(SYSTEMIMAGE)config.xml $(DIST)AirNavigator/
	cd $(DIST); zip -9 -T -x "*.git*" "*.svn*" -r $(ZIPSTANDNAME).zip ttsystem AirNavigator/* README.txt ; cd ..;
	@rm $(DIST)ttsystem
	@mv -f $(DIST)config.xml $(DIST)AirNavigator/
