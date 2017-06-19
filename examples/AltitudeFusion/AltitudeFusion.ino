/* 
   AltitudeFusion.ino: Altitude estimation through baro/accelerometer fusion

   Adapted from

     https://github.com/kriswiner/Teensy_Flight_Controller/blob/master/EM7180_MPU9250_BMP280

    https://github.com/multiwii/baseflight/blob/master/src/imu.c

    https://github.com/multiwii/baseflight/blob/master/src/sensors.c

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

static EM7180 em7180;

#define UPDATE_HZ       20
#define BARO_TAB_SIZE   48
#define CALIBRATION_SEC 8
#define BARO_NOISE_LPF  0.5f


void setup()
{
#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#else
    Wire.begin();
#endif

    delay(100);

    Serial.begin(38400);

    // Start the EM710
    uint8_t status = em7180.begin(8, 2000, 1000);
    while (status) {
        Serial.println(EM7180::errorToString(status));
    }
}

// Pressure in Pascals to altitude in centimeters
static float paToCm(uint32_t pa)
{
    return (1.0f - powf(pa / 101325.0f, 0.190295f)) * 4433000.0f;
}

void loop()
{  
    static uint32_t baroPressureSum;
    static uint32_t millisPrev;
    static int32_t  baroHistTab[BARO_TAB_SIZE];
    static int      baroHistIdx;
    static int32_t  baroGroundPressure;
    static int32_t  baroGroundAltitude;
    static int32_t  BaroAlt;
    uint8_t         indexplus1;
    static uint32_t calibrationStart;

    // Poll EM7180 every iteration
    em7180.poll();

    // Periodically sample baro
    if ((millis() - millisPrev) > 1000/UPDATE_HZ) { 

        // Start baro calibration if not yet started
        if (!calibrationStart) 
            calibrationStart = millis();

        // Grab baro pressure and smooth it using history
        float pressure, temperature;
        em7180.getBaro(pressure, temperature);
        indexplus1 = (baroHistIdx + 1) % BARO_TAB_SIZE;
        baroHistTab[baroHistIdx] = (int32_t)pressure;
        baroPressureSum += baroHistTab[baroHistIdx];
        baroPressureSum -= baroHistTab[indexplus1];
        baroHistIdx = indexplus1;

        // Compute baro ground altitude during calibration
        if (millis() - calibrationStart < 1000*CALIBRATION_SEC) {
            baroGroundPressure -= baroGroundPressure / 8;
            baroGroundPressure += baroPressureSum / (BARO_TAB_SIZE - 1);
            baroGroundAltitude = paToCm(baroGroundPressure/8);
        }

        int32_t BaroAlt_tmp = paToCm((float)baroPressureSum/(BARO_TAB_SIZE-1)); 
        BaroAlt_tmp -= baroGroundAltitude;
        BaroAlt = lrintf((float)BaroAlt * BARO_NOISE_LPF + (float)BaroAlt_tmp * (1.0f - BARO_NOISE_LPF)); // additional LPF to reduce baro noise

        Serial.println(BaroAlt);

        millisPrev = millis(); 
    }
}
