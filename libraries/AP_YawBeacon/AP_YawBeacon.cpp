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

#include "AP_YawBeacon.h"

#if AP_YAWBEACON_ENABLED

#include "AP_YawBeacon_Pin.h"
#include "AP_YawBeacon_SITL.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>

const AP_Param::GroupInfo AP_YawBeacon::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Yaw beacon type
    // @Description: Type of yaw beacon receiver connected
    // @Values: 0:None,1:GPIO pin,10:SITL
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_YawBeacon, _type, (int8_t)Type::NONE, AP_PARAM_FLAG_ENABLE),

    // @Param: PIN
    // @DisplayName: Yaw beacon receiver pin
    // @Description: GPIO pin the receiver output is connected to. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot. A servo output can be used as GPIO by setting its SERVOx_FUNCTION to -1.
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    AP_GROUPINFO("PIN", 2, AP_YawBeacon, _pin, -1),

    // @Param: YAW
    // @DisplayName: Heading at beacon detection
    // @Description: The vehicle's true heading in degrees at the instant the receiver detects the beacon. This is the bearing from the vehicle to the beacon plus any fixed offset of the receiver's boresight from the vehicle's nose. Calibrate by comparing YBCN log messages against a reference heading.
    // @Units: deg
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("YAW", 3, AP_YawBeacon, _yaw_deg, 0),

    // @Param: ACC
    // @DisplayName: Yaw measurement accuracy
    // @Description: 1-sigma accuracy of a beacon yaw measurement, floored at 1 degree. Should account for the receiver's angular acceptance window and pulse timing jitter (1ms of timing jitter at 720 deg/sec of rotation is 0.72 degrees of yaw).
    // @Units: deg
    // @Range: 1 30
    // @User: Advanced
    AP_GROUPINFO("ACC", 4, AP_YawBeacon, _acc_deg, 5),

    // @Param: EDGE
    // @DisplayName: Pulse detection edge
    // @Description: Signal edge marking beacon detection. Demodulating IR receivers (e.g. TSOP series) have active-low outputs so should use Falling.
    // @Values: 0:Rising,1:Falling
    // @User: Standard
    AP_GROUPINFO("EDGE", 5, AP_YawBeacon, _edge, 0),

    // @Param: BLANK
    // @DisplayName: Post-pulse blanking time
    // @Description: Pulses arriving within this interval of the previously accepted pulse are ignored, rejecting reflections and repeated triggering while the beacon remains in the receiver's field of view. Must be shorter than the fastest expected rotation period. Note the EKF accepts yaw measurements at most every 50ms.
    // @Units: ms
    // @Range: 0 2000
    // @User: Advanced
    AP_GROUPINFO("BLANK", 6, AP_YawBeacon, _blank_ms, 100),

    // @Param: DELAY
    // @DisplayName: Receiver latency
    // @Description: Fixed delay between the receiver pointing at the beacon and the output pulse edge, subtracted from the measurement timestamp. Demodulating IR receivers typically add several hundred microseconds. On a fast spinning vehicle a constant unmodelled latency appears as a fixed yaw offset which can instead be folded into YBCN_YAW.
    // @Units: ms
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("DELAY", 7, AP_YawBeacon, _delay_ms, 0),

    AP_GROUPEND
};

AP_YawBeacon *AP_YawBeacon::_singleton;

AP_YawBeacon::AP_YawBeacon()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_YawBeacon must be singleton");
    }
#endif
    _singleton = this;
}

void AP_YawBeacon::init()
{
    if (_backend != nullptr) {
        return;
    }
    switch (Type(_type.get())) {
    case Type::NONE:
        break;
    case Type::PIN:
        _backend = NEW_NOTHROW AP_YawBeacon_Pin(*this);
        break;
#if AP_YAWBEACON_SITL_ENABLED
    case Type::SITL:
        _backend = NEW_NOTHROW AP_YawBeacon_SITL(*this);
        break;
#endif
    }
}

void AP_YawBeacon::update()
{
    if (_backend != nullptr) {
        _backend->update();
    }
}

// called by backends when the receiver detects the beacon
void AP_YawBeacon::handle_pulse(uint64_t pulse_us)
{
    // reject pulses inside the blanking interval (reflections or repeated
    // triggering while the beacon remains in the field of view)
    if (_last_pulse_us != 0 &&
        (pulse_us <= _last_pulse_us ||
         (pulse_us - _last_pulse_us) < (uint64_t)MAX(_blank_ms.get(), 0) * 1000ULL)) {
        return;
    }
    const float dt = (_last_pulse_us == 0) ? 0 : (pulse_us - _last_pulse_us) * 1.0e-6f;
    _last_pulse_us = pulse_us;
    _last_pulse_ms = AP_HAL::millis();
    _pulse_count++;

    // remove the fixed receiver latency to get the time the receiver actually
    // pointed at the beacon. Round to the nearest millisecond; at high spin
    // rates each millisecond of timestamp error is significant yaw error
    const uint64_t delay_us = (uint64_t)(constrain_float(_delay_ms.get(), 0, 100) * 1000);
    const uint32_t meas_time_ms = (uint32_t)((pulse_us - MIN(delay_us, pulse_us) + 500ULL) / 1000ULL);

    // floor the accuracy at 1 degree: the innovation gate is a small multiple
    // of this and in-flight gate rejection has no automatic recovery
    const float yaw_rad = wrap_PI(radians(_yaw_deg.get()));
    const float yaw_err_rad = radians(MAX(_acc_deg.get(), 1.0f));

    AP::ahrs().writeEulerYawAngle(yaw_rad, yaw_err_rad, meas_time_ms);

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: YBCN
    // @Description: Yaw beacon pulse
    // @Field: TimeUS: Time since system startup
    // @Field: PT: system time of the pulse edge
    // @Field: Yaw: configured vehicle heading at beacon detection
    // @Field: DT: interval since the previous pulse
    // @Field: Cnt: total accepted pulse count
    AP::logger().WriteStreaming("YBCN", "TimeUS,PT,Yaw,DT,Cnt",
                                "ssds-", "FF000", "QQffI",
                                AP_HAL::micros64(),
                                pulse_us,
                                _yaw_deg.get(),
                                dt,
                                _pulse_count);
#endif
}

// singleton instance
namespace AP {

AP_YawBeacon *yawbeacon()
{
    return AP_YawBeacon::get_singleton();
}

}

#endif  // AP_YAWBEACON_ENABLED
