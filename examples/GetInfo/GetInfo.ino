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

#include <i2c_t3.h>

EM7180 em7180;

void setup()
{
    // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);

    delay(100);

    Serial.begin(38400);

    // Start the EM710
    uint8_t status = em7180.begin();
    while (status) {
        Serial.println(EM7180::errorToString(status));
    }

    // Report SENtral device information

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
    Serial.println(" (should be: 0x02");

    delay(1000); // give some time to read the screen

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
    Serial.print(em7180.getActualTempRate());
    Serial.println(" Hz"); 
    delay(1000); // give some time to read the screen

}

void loop()
{  
}
