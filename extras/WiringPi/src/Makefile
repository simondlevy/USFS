# 
# Makefile for RPi_EM7180 library
#
#
# Copyright (c) 2016 Simon D. Levy
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
# and associated documentation files (the "Software"), to deal in the Software without restriction, 
# including without limitation the rights to use, copy, modify, merge, publish, distribute, 
# sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or 
# substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
# BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

SRCDIR = ../../../src

all: libem7180.so 

libem7180.so: EM7180.o wiringpi_platform.o
	g++ -shared -o libem7180.so EM7180.o wiringpi_platform.o

EM7180.o: $(SRCDIR)/EM7180.cpp $(SRCDIR)/EM7180.h
	g++ -Wall -fpic -std=c++11 -c $(SRCDIR)/EM7180.cpp

wiringpi_platform.o: wiringpi_platform.cpp $(SRCDIR)/cross_platform.h
	g++ -Wall -fpic -std=c++11 -c -I$(SRCDIR) wiringpi_platform.cpp

install: libem7180.so
	cp libem7180.so /usr/lib
	cp $(SRCDIR)/EM7180.h /usr/local/include

clean:
	rm -f *.o *.so *~ 
