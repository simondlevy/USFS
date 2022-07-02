#pragma once

#include <stdint.h>

bool usfs2_begin(void);
void usfs2_enableEvents(uint8_t usfs2_mask);
void usfs2_setAccelLpfBandwidth(uint8_t usfs2_bw);
void usfs2_setAccelRate(uint8_t usfs2_rate);
void usfs2_setBaroRate(uint8_t usfs2_rate);
void usfs2_setGyroFs(uint16_t gyro_fs);
void usfs2_setGyroLpfBandwidth(uint8_t usfs2_bw);
void usfs2_setGyroRate(uint8_t usfs2_rate);
void usfs2_setMagAccFs(uint16_t mag_fs, uint16_t acc_fs);
void usfs2_setMagRate(uint8_t usfs2_rate);
void usfs2_setQRateDivisor(uint8_t usfs2_divisor);
