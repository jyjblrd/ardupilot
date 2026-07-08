/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_YawBeacon_config.h"

#if AP_YAWBEACON_ENABLED

#include "AP_YawBeacon.h"

class AP_YawBeacon_Backend {
public:
    AP_YawBeacon_Backend(AP_YawBeacon &frontend) :
        _frontend(frontend)
    {}

    virtual ~AP_YawBeacon_Backend() {}

    // gather pulses from the receiver and report them to the frontend
    virtual void update() = 0;

protected:
    AP_YawBeacon &_frontend;
};

#endif  // AP_YAWBEACON_ENABLED
