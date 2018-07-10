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

static const uint8_t  ARES           = 8;    // Gs
static const uint16_t GRES           = 2000; // degrees per second
static const uint16_t MRES           = 1000; // microTeslas
static const uint8_t  MAG_RATE       = 100;  // Hz
static const uint16_t ACCEL_RATE     = 330;  // Hz
static const uint16_t GYRO_RATE      = 330;  // Hz
static const uint8_t  BARO_RATE      = 50;   // Hz
static const uint8_t  Q_RATE_DIVISOR = 5;    // 1/5 gyro rate
 
EM7180Master em7180 = EM7180Master(ARES, GRES, MRES, MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);

void setup()
{
    // Start I^2C
#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#else
    Wire.begin();
#endif

    delay(100);

    Serial.begin(115200);

    // Start the EM7180 in master mode
    if (!em7180.begin()) {

        while (true) {
            Serial.println(em7180.getErrorString());
        }
    }
}

static void report(const char * label, uint32_t & count, const char * delim, uint16_t actual=0)
{
    Serial.print(label);
    Serial.print(": ");
    Serial.print(count);
    Serial.print("Hz");

    if (actual > 0) {
        Serial.print(" [");
        Serial.print(actual);
        Serial.print("]");
    }

    Serial.print(delim);

    count = 0;
}

void loop()
{  
    static uint32_t q, a, g, m, b;

    em7180.checkEventStatus();

    if (em7180.gotError()) {
        Serial.print("ERROR: ");
        Serial.println(em7180.getErrorString());
        return;
    }

    if (em7180.gotQuaternion()) {
        float qw, qx, qy, qz;
        em7180.readQuaternion(qw, qx, qy, qz);
        q++;
    }

    if (em7180.gotAccelerometer()) {
        int16_t ax, ay, az;
        em7180.readAccelerometer(ax, ay, az);
        a++;
    }

    if (em7180.gotGyrometer()) {
        int16_t gx, gy, gz;
        em7180.readGyrometer(gx, gy, gz);
        g++;
    }

    if (em7180.gotMagnetometer()) {
        int16_t mx, my, mz;
        em7180.readMagnetometer(mx, my, mz);
        m++;
    }

    if (em7180.gotBarometer()) {
        float pressure, temperature; em7180.readBarometer(pressure, temperature);
        b++;
    }

    static uint32_t start;

    if (micros() - start > 1e6) {
        b /= 2; // need two cycles for complete baro reading (temperature, pressure)
        report("G", g, "\t", em7180.getActualGyroRate());
        report("A", a, "\t", em7180.getActualAccelRate());
        report("M", m, "\t", em7180.getActualMagRate());
        report("B", b, "\t", em7180.getActualBaroRate());
        report("Q", q, "\n");
        start = micros();
    }
}


