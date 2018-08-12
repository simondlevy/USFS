# 
# Makefile for EM7180 WiringPi examples
#
# Copyright (c) 2018 Simon D. Levy
#
# This file is part of EM7180.
#
# EM7180 is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# EM7180 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with EM7180.  If not, see <http://www.gnu.org/licenses/>.

  
# Change this to whereever you installed CrossPlatformI2C
CPI2C = $(HOME)/CrossPlatformI2C/

ALL = MasterTest

EMSRC = ../../src

I2CINC = $(CPI2C)/src
I2CSRC = $(CPI2C)/extras/wiringpi/src

all: $(ALL)

MasterTest: MasterTest.o EM7180.o WiringPiI2C.o main.o
	g++ -std=c++11 -o MasterTest MasterTest.o EM7180.o WiringPiI2C.o main.o -lwiringPi

MasterTest.o: MasterTest.cpp 
	g++ -std=c++11 -Wall -I$(I2CINC) -I$(EMSRC) -c MasterTest.cpp

EM7180.o: $(EMSRC)/EM7180.cpp 
	g++ -std=c++11 -Wall -I$(I2CINC) -I$(EMSRC) -c $(EMSRC)/EM7180.cpp

WiringPiI2C.o: $(I2CSRC)/WiringPiI2C.cpp 
	g++ -std=c++11 -Wall -I$(I2CINC) -c $(I2CSRC)/WiringPiI2C.cpp

main.o: main.cpp 
	g++ -std=c++11 -Wall -c main.cpp

run: GetInfo
	./GetInfo

edit:
	vim GetInfo.cpp

clean:
	rm -f $(ALL) *.o *~
