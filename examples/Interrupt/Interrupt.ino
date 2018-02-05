/* 
   Interrupt.ino: Example sketch for running EM7180 SENtral sensor hub with interrupts.

   Adapted from

     https://raw.githubusercontent.com/kriswiner/Teensy_Flight_Controller/master/em7180.MPU9250_BMP280

     https://github.com/kriswiner/EM7180_SENtral_sensor_hub/wiki/E.-Polling-versus-Interrupts:-Reading-Data-from-the-EM7180

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

#ifdef __MK20DX256__
// Teensy
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
// Other Arduino
#include <Wire.h>
#define NOSTOP false
#endif

#include <EM7180.h>

EM7180 em7180;

// D12 is EM7180 interrupt pin on Ladybug Flight Controller.
// Set it to something else for other boards as needed.
static const uint8_t intPin = 12;

void setup()
{

#ifdef __MK20DX256__
    // Teensy
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#else
    // Other Arduino
    Wire.begin();
#endif

    Serial.begin(115200);

    // Start the EM710 in interrupt mode
    uint8_t status = em7180.begin(8, 2000, 1000, intPin);

    while (status) {
        Serial.println(em7180.getErrorString());
    }
}


void loop()
{  
    /*
    if (em7180.gotInterrupt()) {

        Serial.println("yes");

    }

    else {
        Serial.println("no");
    }*/
}
