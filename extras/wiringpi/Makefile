# 
# Makefile for USFS WiringPi examples
#
# Copyright (c) 2018 Simon D. Levy
#
# This file is part of USFS.
#
# USFS is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# USFS is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with USFS.  If not, see <http://www.gnu.org/licenses/>.

  
# Change this to whereever you installed CrossPlatformI2C
CPDB = $(HOME)/CrossPlatformDataBus

ALL = MasterTest

EMSRC = ../../src

CPSRC  = $(CPDB)/src
CPEXT  = $(CPDB)/extras
CPCMN  = $(CPEXT)/common
I2CSRC = $(CPEXT)/i2c/wiringpi/src

all: $(ALL)

MasterTest: MasterTest.o USFS.o USFS_Master.o WiringPiI2C.o main.o
	g++ -std=c++11 -o MasterTest MasterTest.o USFS.o USFS_Master.o WiringPiI2C.o main.o -lwiringPi

MasterTest.o: MasterTest.cpp 
	g++ -std=c++11 -Wall -I$(CPSRC) -I$(EMSRC) -c MasterTest.cpp

USFS.o: $(EMSRC)/USFS.cpp 
	g++ -std=c++11 -Wall -I$(CPSRC) -I$(EMSRC) -c $(EMSRC)/USFS.cpp

USFS_Master.o: $(EMSRC)/USFS_Master.cpp 
	g++ -std=c++11 -Wall -I$(CPSRC) -I$(EMSRC) -c $(EMSRC)/USFS_Master.cpp

WiringPiI2C.o: $(I2CSRC)/WiringPiI2C.cpp 
	g++ -std=c++11 -Wall -I$(CPSRC) -c $(I2CSRC)/WiringPiI2C.cpp

main.o: $(CPCMN)/main.cpp 
	g++ -std=c++11 -Wall -c $(CPCMN)/main.cpp

run: GetInfo
	./GetInfo

edit:
	vim GetInfo.cpp

clean:
	rm -f $(ALL) *.o *~
