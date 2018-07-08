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

  
ALL = GetInfo

EMSRC = ../../../src
RPISRC = ../src

all: $(ALL)

GetInfo: GetInfo.o EM7180.o wiringpi_platform.o
	g++ -std=c++11 -o GetInfo GetInfo.o EM7180.o wiringpi_platform.o -lwiringPi

GetInfo.o: GetInfo.cpp 
	g++ -std=c++11 -Wall -I$(RPISRC) -I$(EMSRC) -c GetInfo.cpp

EM7180.o: $(EMSRC)/EM7180.cpp 
	g++ -std=c++11 -Wall -I$(EMSRC) -c $(EMSRC)/EM7180.cpp

wiringpi_platform.o: $(RPISRC)/wiringpi_platform.cpp 
	g++ -std=c++11 -Wall -I$(RPISRC) -I$(EMSRC) -c $(RPISRC)/wiringpi_platform.cpp

run: GetInfo
	./GetInfo

edit:
	vim GetInfo.cpp

clean:
	rm -f $(ALL) *.o *~
