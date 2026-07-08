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

#if AP_YAWBEACON_SITL_ENABLED

#include "AP_YawBeacon_Backend.h"

/*
  SITL backend. SITL does not simulate GPIO interrupts, so the beacon pulse is
  synthesized directly: a pulse is generated whenever the simulator's true yaw
  sweeps through the configured beacon heading (YBCN_YAW), with the pulse time
  interpolated between simulation samples.
 */
class AP_YawBeacon_SITL : public AP_YawBeacon_Backend {
public:
    using AP_YawBeacon_Backend::AP_YawBeacon_Backend;

    void update() override;

private:
    bool have_prev;
    float prev_yaw_deg;     // simulator true yaw at the previous update
    uint64_t prev_time_us;  // system time of the previous update
};

#endif  // AP_YAWBEACON_SITL_ENABLED
