/* 
   Interrup.cpp: Example WiringPi sketch for running EM7180 SENtral sensor hub in master mode with interrupts

   When an interrupt is detected, we check for accelerometer data and report it.

   Copyright (c) Simon D. Levy 2018

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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <wiringPi.h>


static const uint8_t  ARES           = 8;    // Gs
static const uint16_t GRES           = 2000; // degrees per second
static const uint16_t MRES           = 1000; // microTeslas
static const uint8_t  MAG_RATE       = 100;  // Hz
static const uint16_t ACCEL_RATE     = 200;  // Hz
static const uint16_t GYRO_RATE      = 200;  // Hz
static const uint8_t  BARO_RATE      = 50;   // Hz
static const uint8_t  Q_RATE_DIVISOR = 3;    // 1/3 gyro rate
 
EM7180_Master em7180 = EM7180_Master(ARES, GRES, MRES, MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);

// Use GPIO Pin 17, which is Pin 0 for wiringPi library
static const uint8_t INTERRUPT_PIN = 0;

static bool newData;

static void interruptHandler(void)
{
    newData = true;
}

void setup()
{
    // Set up the wiringPi library
    if (wiringPiSetup () < 0) {
        fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
        exit(1);
    }

    // Set up the interrupt
    pinMode(INTERRUPT_PIN, INPUT);
    wiringPiISR(pin, INT_EDGE_RISING, interruptHandler) ;

    // Start the EM7180 in master mode
    if (!em7180.begin()) {

        while (true) {
            fprintf(stderr, "%s\n", em7180.getErrorString());
        }
    }
}

void loop()
{  
    if (newData) {

        newData = false;

        em7180.checkEventStatus();

        if (em7180.gotError()) {
            fprintf(stderr, "%s\n", em7180.getErrorString());
            return;
        }

        else if (em7180.gotAccelerometer()) {
            int16_t ax, ay, az;
            em7180.readAccelerometer(ax, ay, az);
            printf("Accel: %d, %d, %d\n", ax, ay, az);
        }
    }
}
