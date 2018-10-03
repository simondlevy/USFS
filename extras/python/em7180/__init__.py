'''
   Python classes for EM7180 SENtral Sensor

   Copyright (C) 2018 Simon D. Levy

   This file is part of EM7180.

   EM7180 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   EM7180 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
'''

class EM7180(object):

    def __init__(self):

        return

class EM7180_Master(object):

    def __init__(self, magRate, accelRate, gyroRate, baroRate, qRateDivisor):

        self.magRate = magRate
        self.accelRate = accelRate
        self.gyroRate = gyroRate 
        self.baroRate = baroRate
        self.qRateDivisor = qRateDivisor

        self.eventStatus = 0

        self.em7180 = EM7180()

    def begin(self):

        return True

    def getErrorString(self):

        return ''

    def checkEventStatus(self):

        return

    def gotError(self):

        return False

    def gotQuaternion(self):

        return True

    def readQuaternion(self):

        return 0,0,0,0

    def gotAccelerometer(self):

        return True

    def readAccelerometer(self):

        return 0,0,0

    def gotGyrometer(self):

        return True

    def readGyrometer(self):

        return 0,0,0

    def gotMagnetometer(self):

        return True

    def readMagnetometer(self):

        return 0,0,0

    def gotBarometer(self):

        return True

    def readBarometer(self):

        return 0,0


