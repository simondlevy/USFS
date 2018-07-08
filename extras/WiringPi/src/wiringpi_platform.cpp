/* 
   wiringpi_platform.cpp: WiringPi implementation of cross-platform routines

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

#include "cross_platform.h"

#include <wiringPi.h>
#include <wiringPiI2C.h>

void _delay(uint32_t msec)
{
    delay(msec);
}

void _pinModeInput(uint8_t pin)
{
    pinMode(pin, INPUT);
}

void _attachRisingInterrupt(uint8_t pin, void (*isr)(void))
{
    wiringPiISR(pin, isr, EDGE_RISING);
}

uint8_t _i2c_setup(uint8_t address)
{
    return (uint8_t)wiringPiI2CSetup (int devId);
}

void _i2c_writeRegister(uint8_t address, uint8_t subAddress, uint8_t data)
{
	wiringPiI2CWriteReg8(address, subAddress, data);
}

void _i2c_readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    for (uint8_t i=0; i<count; ++i) {
        dest[i] = wiringPiI2CReadReg8(address, subAddress+i);
    }
}
