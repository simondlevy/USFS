/* 
   USFS pass-through example

   Puts USFS in pass-through mode and reports I^2C addresses found

   Copyright (C) 2023 Simon D. Levy

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

#include <Wire.h>

#include "usfs.hpp"

static Usfs usfs;

void setup()
{
    Serial.begin(115200);

    Wire.begin(); 
    Wire.setClock(400000); 
    delay(100);

    usfs.setPassThroughMode();
}

void loop()
{
    Serial.print("Scanning for I^2C devices ...");

    int nDevices = 0;

    for(byte address = 1; address < 127; address++ ) 
    {
        Wire.beginTransmission(address);

        byte error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address<16) 
                Serial.print("0");
            Serial.println(address,HEX);

            nDevices++;
        }
        else if (error==4) 
        {
            Serial.print("Unknown error at address 0x");
            if (address<16) 
                Serial.print("0");
            Serial.println(address,HEX);
        }    
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n"); 

    delay(1000);
}
