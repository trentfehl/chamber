#*********************************************************************
# This is the makefile for chamber application and its dependencies
# *********************************************************************

# Makefile dir
ROOT_DIR:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

CCFLAGS=-std=c++14 -Wall -Ofast -pthread
SUPPORT=./mcp3008/mcp3008Spi.cpp ./pid/pid.cpp

prefix := /usr/local/

# define all programs
PROGRAMS = application 
SOURCES = ${PROGRAMS:=.cpp} 

CXX=g++
CFLAGS=${CCFLAGS}

all: ${PROGRAMS}

${PROGRAMS}: ${SOURCES}
	$(CXX) $(CFLAGS) -lArduiPi_OLED -lpigpio $(SUPPORT) $@.cpp -o $@.exe

clean:
	rm -rf $(PROGRAMS)
