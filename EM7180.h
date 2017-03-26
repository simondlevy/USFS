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

#include <i2c_t3.h>

class _EM7180 {

    protected:

        void begin(void);
};

class EM7180 : public _EM7180 {

    public:

        void begin(void);

        void loop(void);
};

class EM7180_Passthru : public _EM7180 {

    public:

        bool begin(void);
};
