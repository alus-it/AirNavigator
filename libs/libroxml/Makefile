# ============================================================================
#  Name        : Makefile
#  Since       : 11/07/2012
#  Author      : Alberto Realis-Luc <alberto.realisluc@gmail.com>
#  Web         : http://www.alus.it/airnavigator/
#  Copyright   : (C) 2010-2013 Alberto Realis-Luc
#  License     : GNU GPL v2
#  Last change : 4/12/2011
#  Description : Simple Makefile to compile roxml library for ARM 
# ============================================================================

CC = arm-linux-gcc
WARN_CFLAGS = -std=gnu99 -pedantic -Wall -Wpointer-arith -Wcast-qual -Wstrict-prototypes -Wno-unused-parameter -Werror
CFLAGS = -c -O3 -fPIC -mcpu=arm920t $(WARN_CFLAGS)

all: libroxml.so

libroxml.so: roxml.o roxml-internal.o roxml-parse-engine.o
	@$(CC) -shared -Wl,-soname,$@ -o $@ roxml.o roxml-internal.o roxml-parse-engine.o

roxml.o: roxml-parse-engine.c
	@$(CC) $(CFLAGS) roxml.c -o $@

roxml-internal.o: roxml-parse-engine.c
	@$(CC) $(CFLAGS) roxml-internal.c -o $@

roxml-parse-engine.o: roxml-parse-engine.c
	@$(CC) $(CFLAGS) roxml-parse-engine.c -o $@

clean:
	@rm -rf *.o *.so
