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

void usfs2_algorithmControlReset(void); 

void usfs2_algorithmControlRequestParameterTransfer(void);

bool usfs2_begin(void);

void usfs2_enableEvents(uint8_t usfs2_mask);

uint8_t usfs2_getAlgorithmStatus(void);

uint8_t usfs2_getErrorStatus(void);

uint8_t usfs2_getEventStatus(void);

uint8_t usfs2_getParamAcknowledge(void);

uint8_t usfs2_getPassThruStatus(void);

uint8_t usfs2_getProductId(void); 

uint16_t usfs2_getRamVersion(void);

uint8_t usfs2_getRevisionId(void); 

uint16_t usfs2_getRomVersion(void);

uint8_t usfs2_getRunStatus(void);

uint8_t usfs2_getSensorStatus(void);

uint8_t usfs2_getSentralStatus(void);

void usfs2_loadParamByte0(uint8_t usfs2_value);
void usfs2_loadParamByte1(uint8_t usfs2_value);
void usfs2_loadParamByte2(uint8_t usfs2_value);
void usfs2_loadParamByte3(uint8_t usfs2_value);

void usfs2_readAccelerometer(int16_t & ax, int16_t & ay, int16_t & az);

void usfs2_readQuaternion(float & qw, float & qx, float & qy, float & qz);

uint8_t usfs2_readSavedParamByte0(void);
uint8_t usfs2_readSavedParamByte1(void);
uint8_t usfs2_readSavedParamByte2(void);
uint8_t usfs2_readSavedParamByte3(void);

void usfs2_requestParamRead(uint8_t usfs2_param);

void usfs2_requestReset(void);

void usfs2_setAccelLpfBandwidth(uint8_t usfs2_bw);

void usfs2_setAccelRate(uint8_t usfs2_rate);

void usfs2_setBaroRate(uint8_t usfs2_rate);

void usfs2_setGyroFs(uint16_t gyro_fs);

void usfs2_setGyroLpfBandwidth(uint8_t usfs2_bw);

void usfs2_setGyroRate(uint8_t usfs2_rate);

void usfs2_setMagAccFs(uint16_t mag_fs, uint16_t acc_fs);

void usfs2_setMagRate(uint8_t usfs2_rate);

void usfs2_setMasterMode(void);

void usfs2_setPassThroughMode(void);

void usfs2_setQRateDivisor(uint8_t usfs2_divisor);

void usfs2_setRunDisable(void);

void usfs2_setRunEnable(void);

void usfs2_writeByte(uint8_t subAddress, uint8_t data);
