/* 
   GetInfo.ino: Report info on EM7180 SENtral sensor hub

   Adapted from

     https://raw.githubusercontent.com/kriswiner/Teensy_Flight_Controller/master/em7180.MPU9250_BMP280

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
#else
#include <Wire.h>
#endif


static const uint8_t  ARES           = 8;    // Gs
static const uint16_t GRES           = 2000; // radians per second
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

    // Start the EM7180 in master mode
    if (!em7180.begin()) {

        while (true) {
            Serial.println(em7180.getErrorString());
        }
    }    
}

void loop()
{  
    if (em7180.hasBaro())        Serial.println("A barometer is installed.");
    if (em7180.hasHumidity())    Serial.println("A humidity sensor is installed.");
    if (em7180.hasTemperature()) Serial.println("A temperature sensor is installed.");
    if (em7180.hasCustom1())     Serial.println("A custom sensor is installed.");
    if (em7180.hasCustom2())     Serial.println("A second custom sensor is installed.");
    if (em7180.hasCustom3())     Serial.println("A third custom sensor is installed.");

    Serial.print("EM7180 ROM Version: 0x");
    Serial.print(em7180.getRomVersion(), HEX);
    Serial.println(" (should be: 0xE609)");

    Serial.print("EM7180 RAM Version: 0x");
    Serial.println(em7180.getRamVersion(), HEX);

    Serial.print("EM7180 ProductID: 0x");
    Serial.print(em7180.getProductId(), HEX);
    Serial.println(" (should be: 0x80)");

    Serial.print("EM7180 RevisionID: 0x");
    Serial.print(em7180.getRevisionId(), HEX);
    Serial.println(" (should be: 0x2");

    Serial.print("Actual MagRate = ");
    Serial.print(em7180.getActualMagRate());
    Serial.println(" Hz"); 
    Serial.print("Actual AccelRate = ");
    Serial.print(10*em7180.getActualAccelRate());
    Serial.println(" Hz"); 
    Serial.print("Actual GyroRate = ");
    Serial.print(10*em7180.getActualGyroRate());
    Serial.println(" Hz"); 
    Serial.print("Actual BaroRate = ");
    Serial.print(em7180.getActualBaroRate());
    Serial.println(" Hz"); 
    Serial.print("Actual TempRate = ");
    Serial.print(em7180.getActualTempRate());
    Serial.println(" Hz"); 

    Serial.print("Run status: ");
    Serial.println(em7180.runStatusNormal() ? "normal" : "other");

    if (em7180.algorithmStatusStandby())                 Serial.println("EM7180 standby status");
    if (em7180.algorithmStatusSlow())                    Serial.println("EM7180 algorithm slow");
    if (em7180.algorithmStatusStillness())               Serial.println("EM7180 in stillness mode");
    if (em7180.algorithmStatusMagCalibrationCompleted()) Serial.println("EM7180 mag calibration completed");
    if (em7180.algorithmStatusMagneticAnomalyDetected()) Serial.println("EM7180 magnetic anomaly detected");
    if (em7180.algorithmStatusUnreliableData())          Serial.println("EM7180 unreliable sensor data");

    uint8_t  accelFs=0;
    uint16_t gyroFs=0;
    uint16_t magFs=0;
    em7180.getFullScaleRanges(accelFs, gyroFs, magFs);

    Serial.print("Magnetometer Full Scale Range: +/-");
    Serial.print(magFs);
    Serial.println("uT");
    Serial.print("Accelerometer Full Scale Range: +/-");
    Serial.print(accelFs);
    Serial.println("g");
    Serial.print("Gyroscope Full Scale Range: +/-");
    Serial.print(gyroFs);
    Serial.println("dps");

    static uint32_t count;
    Serial.print(count++);
    Serial.println(" ----------------------------------------------");

    delay(1000);
}
