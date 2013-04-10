###############################################################
#
# Purpose: Makefile for "read_write"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################
DEBUG=yes
ifeq ($(DEBUG),yes)
	CFLAGS=-c -Wall -ansi -pedantic -g
	LDFLAGS=
else
	CFLAGS=-c -Wall -ansi -pedantic
	LDFLAGS=
	
endif
CC=g++
SOURCES=main.cpp TestMotion.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=test

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@
