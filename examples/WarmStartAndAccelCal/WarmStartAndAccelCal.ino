/* 
   IMU calibration sketch for USFS on Teensy boards

   Copyright (C) 2019 Simon D. Levy

   This file is part of USFS.

   USFS is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   USFS is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with USFS.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "UsfsWarmStartAndAccelCal.h"

void setup(void)
{
#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#else
    Wire.begin();
#endif

    usfs_warm_start_and_accel_cal_setup();
}

void loop(void)
{
    usfs_warm_start_and_accel_cal_loop();
}


