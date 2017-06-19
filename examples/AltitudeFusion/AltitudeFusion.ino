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
#include "Altitude.hpp"

#ifdef __MK20DX256__
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

#define ESTIMATOR_UPDATE_MILLIS 50
#define IMU_UPDATE_MICROS       3500

static EM7180 em7180;
static Altitude alti;

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

void loop()
{  
    // Poll EM7180 every iteration
    em7180.poll();

    // Periodically use baro to get estimate
    static uint32_t estimatorMillisPrev;
    if ((millis() - estimatorMillisPrev) > ESTIMATOR_UPDATE_MILLIS) { 

        float pressure, temperature;
        em7180.getBaro(pressure, temperature);

        alti.updateBaro(pressure);

        estimatorMillisPrev = millis(); 
    }

    // More frequently, update with accelerometer and Euler angles
    static uint32_t imuMicrosPrev;
    if ((micros() - imuMicrosPrev) > IMU_UPDATE_MICROS) { 

        int16_t accelRaw[3];
        em7180.getAccelRaw(accelRaw[0], accelRaw[1], accelRaw[2]);

        float q[4];
        em7180.getQuaternions(q);
        float eulerAngles[3];
        eulerAngles[0] = atan2(2.0f * (q[3] * q[0] + q[1] * q[2]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2]);  // roll
        eulerAngles[1] = asin(2.0f * (q[0] * q[2] - q[3] * q[1]));                                                          // pitch
        eulerAngles[2] = atan2(2.0f * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2]);  // yaw 

        alti.updateImu(accelRaw, eulerAngles);

        imuMicrosPrev = micros(); 
    }


}
