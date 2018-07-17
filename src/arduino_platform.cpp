/* 

   arduino_platform.cpp: Arduino implementation of cross-platform routines

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

#include <Arduino.h>

/* For Teensy use i2c_t3 library and I2C_NOSTOP
   Teensy 3.0 (__MK20DX128__)
   Teensy 3.1/3.2 (__MK20DX256__)
   Teensy LC (__MKL26Z64__)
   Teensy 3.5 (__MK64FX512__)
   Teensy 3.6 (__MK66FX1M0__)
*/
#if defined(CORE_TEENSY)   && (  \
    defined(__MK20DX128__) ||    \
    defined(__MK20DX256__) ||    \
    defined(__MKL26Z64__)  ||    \
    defined(__MK64FX512__) ||    \
    defined(__MK66FX1M0__)    )
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

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
    attachInterrupt(pin, isr, RISING);
}

uint8_t _i2c_setup(uint8_t address)
{
    return address;
}

void _i2c_writeRegister(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

void _i2c_readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(NOSTOP);      // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
    while (Wire.available()) {
        dest[i++] = Wire.read(); 
    }         
}
