/* 
   Header file for USFS calibration sketches

   Copyright (C) 2019 Simon D. Levy

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

#pragma once

// Support Teensy
#ifdef __MK20DX256__
#include <i2c_t3.h>
#else
#include <Wire.h>
#endif

#include "USFS_old.h"

void usfs_warm_start_and_accel_cal_setup(void);

void usfs_warm_start_and_accel_cal_loop(void);
