/* 
   Interrup.ino: Example sketch for running EM7180 SENtral sensor hub in master mode with interrupts

   When an interrupt is detected, we check for accelerometer data and report it.

   Adapted from

     https://github.com/kriswiner/EM7180_SENtral_sensor_hub/tree/master/WarmStartandAccelCal

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

// This works for LadybugFC; you should change it for your controller.
static const uint8_t INTERRUPT_PIN = 12;

extern volatile bool newData;

static const uint8_t  ARES           = 8;    // Gs
static const uint16_t GRES           = 2000; // degrees per second
static const uint16_t MRES           = 1000; // microTeslas
static const uint8_t  MAG_RATE       = 100;  // Hz
static const uint16_t ACCEL_RATE     = 200;  // Hz
static const uint16_t GYRO_RATE      = 200;  // Hz
static const uint8_t  BARO_RATE      = 50;   // Hz
static const uint8_t  Q_RATE_DIVISOR = 3;    // 1/3 gyro rate
 
EM7180 em7180 = EM7180(ARES, GRES, MRES, MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);

void setup()
{
#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#else
    Wire.begin();
#endif

    delay(100);

    Serial.begin(115200);

    // Start the EM7180 in master mode with interrupt
    if (!em7180.begin(INTERRUPT_PIN)) {

        while (true) {
            Serial.println(em7180.getErrorString());
        }
    }
}

void loop()
{  
    if (em7180.gotInterrupt()) {

        em7180.checkEventStatus();

        if (em7180.gotError()) {
            Serial.print("ERROR: ");
            Serial.println(em7180.getErrorString());
            return;
        }

        else if (em7180.gotAccelerometer()) {
            int16_t ax, ay, az;
            em7180.readAccelerometer(ax, ay, az);
            Serial.print("Accel: ");
            Serial.print(ax);
            Serial.print(", ");
            Serial.print(ay);
            Serial.print(", ");
            Serial.println(az);
        }
    }
}
