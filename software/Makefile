#*********************************************************************
# This is the makefile for 
# *********************************************************************

# Makefile dir
ROOT_DIR:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

CCFLAGS=-std=c++14 -Wall -Ofast -pthread

prefix := /usr/local/

# define all programs
PROGRAMS = application 
SOURCES = ${PROGRAMS:=.cpp} 

CXX=g++
CFLAGS=${CCFLAGS}

all: ${PROGRAMS}

${PROGRAMS}: ${SOURCES}
	$(CXX) $(CFLAGS) -lArduiPi_OLED ./mcp3008/mcp3008Spi.cpp $@.cpp -o $@.exe

clean:
	rm -rf $(PROGRAMS)
