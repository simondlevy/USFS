#!/usr/bin/env python3

'''
setup.py - Python distutils setup file for EM7180 package.

Copyright (C) 2018 Simon D. Levy

This file is part of EM7180.

EM7180 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

EM7180 is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU Lesser General Public License 
along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
'''

from setuptools import setup

setup (name = 'EM7180',
    version = '0.1',
    description = 'Python library for the EM7180 SENtral Sensor Fusion Solution',
    packages = ['em7180'],
    author='Simon D. Levy',
    author_email='simon.d.levy@gmail.com',
    url='https://github.com/simondlevy/EM7180',
    license='GPL',
    platforms='Raspbian',
    long_description = 'Provides quaternion, IMU, and barometer data'
    )
