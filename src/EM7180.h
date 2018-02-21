/* 

   EM7180.h: Class header for EM7180 SENtral Sensor

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

#include <stdint.h>

class _EM7180 {

    private:

        static bool hasFeature(uint8_t features);

    protected:

        bool begin(void);

        uint8_t errorStatus;

        static void M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, 
                uint8_t count, uint8_t * dest);

        static void    readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
        static uint8_t readByte(uint8_t address, uint8_t subAddress);
        static void    writeByte(uint8_t address, uint8_t subAddress, uint8_t data);


    public:

        const char * getErrorString(void);

        uint8_t getProductId(void); 
        uint8_t getRevisionId(void); 
        uint16_t getRamVersion(void);
        uint16_t getRomVersion(void);

        bool hasBaro(void);
        bool hasHumidity(void);
        bool hasTemperature(void);
        bool hasCustom1(void);
        bool hasCustom2(void);
        bool hasCustom3(void);
};

class EM7180 : public _EM7180 {

    private:

        uint8_t _eventStatus;

        static bool algorithmStatus(uint8_t status);

        static void setGyroFs(uint16_t gyro_fs);
        static void setMagAccFs(uint16_t mag_fs, uint16_t acc_fs);
        static void setIntegerParam (uint8_t param, uint32_t param_val);

        static float uint32_reg_to_float (uint8_t *buf);

    public:

        // Sensible defaults for sensor ranges
        uint8_t  aRes = 8;    // Gs
        uint16_t gRes = 2000; // radians per second
        uint16_t mRes = 1000; // microTeslas

        // Sensible defaults for sensor Output Data Rates (ODRs)
        uint8_t  magRate      = 100; // Hz
        uint16_t accelRate    = 200; // Hz
        uint16_t gyroRate     = 200; // Hz
        uint8_t  baroRate     = 50;  // Hz
        uint8_t  qRateDivisor = 3;   // 1/3 gyro rate

        EM7180(void);

        bool begin(int8_t pin=-1);

        void checkEventStatus(void);

        bool gotError(void);

        bool gotInterrupt(void);

        bool gotQuaternions(void);

        bool gotMagnetometer(void);

        bool gotAccelerometer(void);

        bool gotGyrometer(void);

        bool gotBarometer(void);

        void readMagnetometer(int16_t mag[3]);

        void readAccelerometer(int16_t accel[3]);

        void readGyrometer(int16_t gyro[3]);

        void readBarometer(float & pressure, float & temperature);

        void readQuaternions(float q[4]);

        void getBarometer(float & press, float & temp);

        uint8_t getActualMagRate();

        uint16_t getActualAccelRate();

        uint16_t getActualGyroRate();

        uint8_t getActualBaroRate();

        uint8_t getActualTempRate();

        bool runStatusNormal(void);

        bool algorithmStatusStandby(void);

        bool algorithmStatusSlow(void);

        bool algorithmStatusStillness(void);

        bool algorithmStatusMagCalibrationCompleted(void);

        bool algorithmStatusMagneticAnomalyDetected(void);

        bool algorithmStatusUnreliableData(void);

        void getFullScaleRanges(uint8_t& accFs, uint16_t& gyroFs, uint16_t& magFs);
};

class EM7180_Passthru : public _EM7180 {

    public:

        bool begin(void);
};
