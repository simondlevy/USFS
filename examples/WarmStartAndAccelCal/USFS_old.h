/* 
   Class header for USFS

   Copyright (C) 2018 Simon D. Levy

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

#pragma once

#include <stdint.h>

class USFS {

    protected:

    public:

        void algorithmControlReset(void); 

        void algorithmControlRequestParameterTransfer(void);

        bool begin(void);

        void enableEvents(uint8_t mask);

        uint8_t getAlgorithmStatus(void);

        uint8_t getErrorStatus(void);

        uint8_t getEventStatus(void);

        uint8_t getParamAcknowledge(void);

        uint8_t getPassThruStatus(void);

        uint8_t  getProductId(void); 

        uint16_t getRamVersion(void);

        uint8_t  getRevisionId(void); 

        uint16_t getRomVersion(void);

        uint8_t getRunStatus(void);

        uint8_t getSensorStatus(void);

        uint8_t  getSentralStatus(void);

        void loadParamByte0(uint8_t value);
        void loadParamByte1(uint8_t value);
        void loadParamByte2(uint8_t value);
        void loadParamByte3(uint8_t value);

        void readAccelerometer(int16_t & ax, int16_t & ay, int16_t & az);

        void readQuaternion(float & qw, float & qx, float & qy, float & qz);

        uint8_t readSavedParamByte0(void);
        uint8_t readSavedParamByte1(void);
        uint8_t readSavedParamByte2(void);
        uint8_t readSavedParamByte3(void);

        void    requestParamRead(uint8_t param);

        void requestReset(void);

        void setAccelLpfBandwidth(uint8_t bw);

        void setAccelRate(uint8_t rate);

        void setBaroRate(uint8_t rate);

        void setGyroFs(uint16_t gyro_fs);

        void setGyroLpfBandwidth(uint8_t bw);

        void setGyroRate(uint8_t rate);

        void setMagAccFs(uint16_t mag_fs, uint16_t acc_fs);

        void setMagRate(uint8_t rate);

        void setMasterMode(void);

        void setPassThroughMode(void);

        void setQRateDivisor(uint8_t divisor);

        void setRunDisable(void);

        void setRunEnable(void);

        void writeGp36(uint8_t value);
        void writeGp37(uint8_t value);
        void writeGp38(uint8_t value);
        void writeGp39(uint8_t value);
        void writeGp40(uint8_t value);
        void writeGp50(uint8_t value);
        void writeGp51(uint8_t value);
        void writeGp52(uint8_t value);
        void writeGp53(uint8_t value);
        void writeGp54(uint8_t value);
        void writeGp55(uint8_t value);
        void writeGp56(uint8_t value);

}; // class USFS
