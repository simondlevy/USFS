#!/usr/bin/env python3
'''
   mastertest.py: Example Python script for running USFS SENtral sensor hub in master mode.

   Copyright (C) 2018 Simon D. Levy

   This file is part of USFS.

   USFS is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   USFS is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with USFS.  If not, see <http://www.gnu.org/licenses/>.
'''

from usfs import USFS_Master

import math
import time

MAG_RATE       = 100  # Hz
ACCEL_RATE     = 200  # Hz
GYRO_RATE      = 200  # Hz
BARO_RATE      = 50   # Hz
Q_RATE_DIVISOR = 3    # 1/3 gyro rate

usfs = USFS_Master(MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR)

# Start the USFS in master mode
if not usfs.begin():
    print(usfs.getErrorString())
    exit(1)

while True:

    usfs.checkEventStatus()

    if usfs.gotError():
        print('ERROR: ' + usfs.getErrorString())
        exit(1)

    # Define output variables from updated quaternion---these are Tait-Bryan
    # angles, commonly used in aircraft orientation.  In this coordinate
    # system, the positive z-axis is down toward Earth.  Yaw is the angle
    # between Sensor x-axis and Earth magnetic North (or true North if
    # corrected for local declination, looking down on the sensor positive
    # yaw is counterclockwise.  Pitch is angle between sensor x-axis and
    # Earth ground plane, toward the Earth is positive, up toward the sky is
    # negative.  Roll is angle between sensor y-axis and Earth ground plane,
    # y-axis up is positive roll.  These arise from the definition of the
    # homogeneous rotation matrix constructed from q.  Tait-Bryan
    # angles as well as Euler angles are non-commutative that is, the get
    # the correct orientation the rotations must be applied in the correct
    # order which for this configuration is yaw, pitch, and then roll.  For
    # more see http://en.wikipedia.org/wiki/Conversion_between_q_and_Euler_angles 
    # which has additional links.

    if (usfs.gotQuaternion()):

        qw, qx, qy, qz = usfs.readQuaternion()

        roll  = math.atan2(2.0 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
        pitch = -math.asin(2.0 * (qx * qz - qw * qy))
        yaw   = math.atan2(2.0 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz)   

        pitch *= 180.0 / math.pi
        yaw   *= 180.0 / math.pi 
        yaw   += 13.8 # Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        if yaw < 0: yaw   += 360.0  # Ensure yaw stays between 0 and 360
        roll  *= 180.0 / math.pi

        print('Quaternion Roll, Pitch, Yaw: %+2.2f %+2.2f %+2.2f' % (roll, pitch, yaw))

    if usfs.gotAccelerometer():

        ax,ay,az = usfs.readAccelerometer()
        
        print('Accel: %+3.3f %+3.3f %+3.3f' % (ax,ay,az))

    if usfs.gotGyrometer():

        gx,gy,gz = usfs.readGyrometer()

        print('Gyro: %+3.3f %+3.3f %+3.3f' % (gx,gy,gz))
    
     #  Or define output variable according to the Android system, where
     #  heading (0 to 360) is defined by the angle between the y-axis and True
     #  North, pitch is rotation about the x-axis (-180 to +180), and roll is
     #  rotation about the y-axis (-90 to +90) In this systen, the z-axis is
     #  pointing away from Earth, the +y-axis is at the 'top' of the device
     #  (cellphone) and the +x-axis points toward the right of the device.

    if usfs.gotBarometer():
    
        pressure, temperature = usfs.readBarometer()

        print('Baro:')
        print('  Altimeter temperature = %2.2f C' % temperature) 
        print('  Altimeter pressure = %2.2f mbar' % pressure) 
        altitude = (1.0 - math.pow(pressure / 1013.25, 0.190295)) * 44330
        print('  Altitude = %2.2f m\n' % altitude) 
    
    time.sleep(.05)
