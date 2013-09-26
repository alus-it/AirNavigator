# ============================================================================
#  Name        : Makefile
#  Since       : 11/07/2012
#  Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
#  Web         : http://www.alus.it/airnavigator/
#  Copyright   : (C) 2010 Alberto Realis-Luc
#  License     : GNU GPL v2
#  Last change : 11/07/2011
#  Description : Simple Makefile to compile roxml library for ARM 
# ============================================================================

CC = arm-linux-gcc
CFLAGS = -c -O3 -fPIC -Wall

all: libroxml.so

libroxml.so: roxml.o roxml-internal.o roxml-parse-engine.o
	$(CC) -shared -Wl,-soname,$@ -o $@ roxml.o roxml-internal.o roxml-parse-engine.o

roxml.o: roxml-parse-engine.c
	$(CC) $(CFLAGS) roxml.c -o $@

roxml-internal.o: roxml-parse-engine.c
	$(CC) $(CFLAGS) roxml-internal.c -o $@

roxml-parse-engine.o: roxml-parse-engine.c
	$(CC) $(CFLAGS) roxml-parse-engine.c -o $@

clean:
	@rm -rf *.o *.so
