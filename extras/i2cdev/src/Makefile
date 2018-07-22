# 
# Makefile for EM7180 i2cdev implementation
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

SRCDIR = ../../../src

all: libem7180.so 

libem7180.so: EM7180.o i2cdev_platform.o
	g++ -shared -o libem7180.so EM7180.o i2cdev_platform.o

EM7180.o: $(SRCDIR)/EM7180.cpp $(SRCDIR)/EM7180.h
	g++ -Wall -fpic -std=c++11 -c $(SRCDIR)/EM7180.cpp

i2cdev_platform.o: i2cdev_platform.cpp $(SRCDIR)/cross_platform.h
	g++ -Wall -fpic -std=c++11 -c -I$(SRCDIR) i2cdev_platform.cpp

install: libem7180.so
	cp libem7180.so /usr/lib
	cp $(SRCDIR)/EM7180.h /usr/local/include

clean:
	rm -f *.o *.so *~ 
