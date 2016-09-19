# ============================================================================
# Name : Makefile
# Since : 22/4/2011
# Author : Alberto Realis-Luc <alberto.realisluc@gmail.com>
# Web : http://www.alus.it/airnavigator/
# Copyright : (C) 2010-2015 Alberto Realis-Luc
# License : GNU GPL v2
# Repository : https://github.com/AirNavigator/AirNavigator.git
# Last change : 19/9/2016
# Description : Makefile of AirNavigator for TomTom devices
# ============================================================================

# Toolchain path
TOOLCHAIN_PATH = /usr/local/cross/gcc-3.3.4_glibc-2.3.2/bin

# Add toolchain location to the path
export PATH := $(TOOLCHAIN_PATH):$(PATH)

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
$(shell mkdir -p $(BIN) >/dev/null)

# Dependencies dir
DEPDIR = $(BIN).d/
$(shell mkdir -p $(DEPDIR) >/dev/null)

# Dependencies flags
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)$*.Td

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
	Ephemerides.c   \
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
$(shell mkdir -p $(LIB) >/dev/null)

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

# Link
$(DIST)AirNavigator/AirNavigator: $(OBJS) $(LIBS)
	@echo Linking all into: $@
	@$(CC) $(LFLAGS) $(OBJS) -L$(LIB) -lroxml -o $@
	@$(STRIP) $@

# Compile
$(BIN)%.o: $(SRC)%.c
$(BIN)%.o: $(SRC)%.c $(DEPDIR)%.d
	@echo 'Compiling: $<'
	@$(CC) $(DEPFLAGS) $(CFLAGS) -I $(INC) -I $(LIBSRC) -c $< -o $@
	@mv -f $(DEPDIR)$*.Td $(DEPDIR)$*.d

# Dependencies directory
$(DEPDIR)%.d: ;
.PRECIOUS: $(DEPDIR)%.d

-include $(patsubst %,$(DEPDIR)%.d,$(basename $(CFILES)))

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
