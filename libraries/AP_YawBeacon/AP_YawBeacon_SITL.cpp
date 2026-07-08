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

#include "AP_YawBeacon_SITL.h"

#if AP_YAWBEACON_SITL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SITL/SITL.h>

void AP_YawBeacon_SITL::update(void)
{
    const auto *sitl = AP::sitl();
    if (sitl == nullptr) {
        return;
    }

    const float yaw_deg = float(sitl->state.yawDeg);
    const uint64_t now_us = AP_HAL::micros64();

    if (!have_prev) {
        prev_yaw_deg = yaw_deg;
        prev_time_us = now_us;
        have_prev = true;
        return;
    }

    // yaw change this sample and the beacon heading's offset from the previous
    // sample, both signed and wrapped so a crossing is a simple interval test.
    // Steps of 90 degrees or more per sample are ambiguous and are skipped
    const float step = wrap_180(yaw_deg - prev_yaw_deg);
    const float offset = wrap_180(_frontend.get_beacon_yaw_deg() - prev_yaw_deg);

    if (!is_zero(step) && fabsf(step) < 90.0f &&
        ((step > 0 && offset >= 0 && offset < step) ||
         (step < 0 && offset <= 0 && offset > step))) {
        // interpolate the time the yaw swept through the beacon heading
        const float frac = offset / step;
        _frontend.handle_pulse(prev_time_us + (uint64_t)(frac * (float)(now_us - prev_time_us)));
    }

    prev_yaw_deg = yaw_deg;
    prev_time_us = now_us;
}

#endif  // AP_YAWBEACON_SITL_ENABLED
