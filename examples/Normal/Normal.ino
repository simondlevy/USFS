/* 
   Normal.ino: Example sketch for running EM7180 SENtral sensor hub in normal (non-pass-through) mode.

   Adapted from

     https://raw.githubusercontent.com/kriswiner/Teensy_Flight_Controller/master/EM7180_MPU9250_BMP280

   This file is part of EM7180.

   EM7180 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   EM7180 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "EM7180.h"

#include <i2c_t3.h>

void setup()
{
    // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);

    delay(100);

    Serial.begin(38400);

    // Start the EM710
    EM7180_begin();
    EM7180_begin2();
}


void loop()
{  
    EM7180_loop();
}


