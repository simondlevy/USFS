/* 
   RateTest.ino: Example of how to adjust Output Data Rates (ODRs) for SENtral sensor 

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

#ifdef __MK20DX256__
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

EM7180 em7180;

void setup()
{
#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#else
    Wire.begin();
#endif

    delay(100);

    Serial.begin(38400);

    // Start the EM7180 in master mode, polling instead of interrupt
    if (!em7180.begin()) {

        while (true) {
            Serial.println(em7180.getErrorString());
        }
    }
}

static void report(const char * label, uint32_t & count, const char * delim)
{
    Serial.print(label);
    Serial.print(": ");
    Serial.print(count);
    Serial.print("Hz");
    Serial.print(delim);

    count = 0;
}

void loop()
{  
    em7180.checkEventStatus();

    if (em7180.gotError()) {
        Serial.print("ERROR: ");
        Serial.println(em7180.getErrorString());
        return;
    }

    static uint32_t q, a, g, m, b;

    if (em7180.gotQuaternions()) {
        q++;
    }

    if (em7180.gotAccelerometer()) {
        a++;
    }

    if (em7180.gotGyrometer()) {
        g++;
    }

    if (em7180.gotMagnetometer()) {
        m++;
    }

    if (em7180.gotBarometer()) {
        b++;
    }

    static uint32_t start;

    if (micros() - start > 1e6) {
        report("Q", q, "    ");
        report("A", a, "    ");
        report("G", g, "    ");
        report("M", m, "    ");
        report("B", b, "\n");
        start = micros();
    }
}


