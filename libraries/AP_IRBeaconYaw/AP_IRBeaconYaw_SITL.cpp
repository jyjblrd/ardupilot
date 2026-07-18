#include "AP_IRBeaconYaw_SITL.h"

#if AP_IRBEACON_YAW_SITL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SITL/SITL.h>

void AP_IRBeaconYaw_SITL::init()
{
    _healthy = AP::sitl() != nullptr;
}

void AP_IRBeaconYaw_SITL::update()
{
    const auto *sitl = AP::sitl();
    if (sitl == nullptr) {
        _healthy = false;
        return;
    }
    _healthy = true;

    const float yaw_deg = float(sitl->state.yawDeg);
    const uint64_t now_us = AP_HAL::micros64();

    if (_pending_pulse_us != 0 && now_us >= _pending_pulse_us) {
        _frontend.handle_pulse(_pending_pulse_us);
        _pending_pulse_us = 0;
    }

    if (!_have_previous) {
        _previous_yaw_deg = yaw_deg;
        _previous_time_us = now_us;
        _have_previous = true;
        return;
    }

    // Signed wrapped values make crossing detection independent of spin direction.
    const float step = wrap_180(yaw_deg - _previous_yaw_deg);
    const float offset = wrap_180(_frontend.get_beacon_yaw_deg() - _previous_yaw_deg);

    if (_pending_pulse_us == 0 && !is_zero(step) && fabsf(step) < 90.0f &&
        ((step > 0.0f && offset >= 0.0f && offset < step) ||
         (step < 0.0f && offset <= 0.0f && offset > step))) {
        const float fraction = offset / step;
        const uint64_t crossing_us = _previous_time_us +
            uint64_t(fraction * float(now_us - _previous_time_us));
        _pending_pulse_us = crossing_us + _frontend.get_delay_us();
        if (now_us >= _pending_pulse_us) {
            _frontend.handle_pulse(_pending_pulse_us);
            _pending_pulse_us = 0;
        }
    }

    _previous_yaw_deg = yaw_deg;
    _previous_time_us = now_us;
}

#endif // AP_IRBEACON_YAW_SITL_ENABLED
