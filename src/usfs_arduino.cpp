/* 
   USFS Arduino implementaiton

   Copyright (C) 2024 Simon D. Levy

https://github.com/kriswiner/SENtral_sensor_hub/tree/master/WarmStartandAccelCal

This file is part of USFS.

USFS is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

USFS is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with USFS.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <Wire.h>

#include <stdarg.h>
#include <stdio.h>

#include "usfs.hpp"

void Usfs::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) 
{
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t Usfs::readByte(uint8_t address, uint8_t subAddress) 
{
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission();
    Wire.requestFrom((int)address, (int)1);
    return Wire.read();
}

void Usfs::readBytes(
        uint8_t address,
        uint8_t subAddress,
        uint8_t count,
        uint8_t * dst)
{  
    Wire.beginTransmission(address);   
    Wire.write(subAddress);
    Wire.endTransmission(false);      
    uint32_t i = 0;
    Wire.requestFrom((int)address, (int)count);
    while (Wire.available()) {
        dst[i++] = Wire.read(); 
    } 
}

void Usfs::delayMsec(const uint32_t msec)
{
    delay(msec);
}

void Usfs::dbgprintf(const char * fmt, ...)
{
    char buf[256] = {};

    va_list ap = {};

    va_start(ap, fmt);
    vsprintf(buf, fmt, ap);
    va_end(ap);

    Serial.print(buf);
}


