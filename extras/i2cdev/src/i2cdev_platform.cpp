/* 
   wiringpi_platform.cpp: Linux i2cdev implementation of cross-platform routines

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

#include "cross_platform.h"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>

static const uint8_t I2C_BUS_NUMBER = 1;

void _delay(uint32_t msec)
{
    struct timespec t;

    t.tv_sec  = msec / 1000;
    t.tv_nsec = msec*1000000 % 1000000000;

    nanosleep(&t, NULL);
}

uint8_t _i2c_setup(uint8_t address)
{
    // Attempt to open /dev/i2c-<NUMBER>
    char fname[32];
    sprintf(fname,"/dev/i2c-%d", I2C_BUS_NUMBER);
    int fd = open(fname, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Unable to open %s\n", fname);
        return 0;
    }

    // Attempt to make this device an I2C slave
    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        fprintf(stderr, "ioctl failed on %s\n", fname);
        return 0;
    }

    return fd;
}

void _i2c_writeRegister(uint8_t address, uint8_t subAddress, uint8_t data)
{
    (void)address;
    (void)subAddress;
    (void)data;
}

void _i2c_readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    (void)address;
    (void)subAddress;
    (void)count;
    (void)dest;

    for (uint8_t i=0; i<count; ++i) {
    }
}
