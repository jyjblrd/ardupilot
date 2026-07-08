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

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class AP_YawBeacon_Backend;

/*
  Driver for a yaw beacon: a receiver on a rotating vehicle which pulses each
  time it points at a stationary beacon. Each pulse is an absolute yaw
  measurement (the vehicle heading at the pulse is the configured YBCN_YAW)
  which is sent to the EKF as an independent yaw angle observation. Select it
  as the EKF yaw source with EK3_SRCn_YAW = 9 (YawBeacon).
 */
class AP_YawBeacon {
public:
    AP_YawBeacon();

    CLASS_NO_COPY(AP_YawBeacon);

    static AP_YawBeacon *get_singleton() { return _singleton; }

    // beacon receiver types
    enum class Type : uint8_t {
        NONE = 0,
        PIN  = 1,
#if AP_YAWBEACON_SITL_ENABLED
        SITL = 10,
#endif
    };

    // allocate the configured backend
    void init();

    // consume pulses from the backend and forward them to the EKF, called
    // regularly from the vehicle scheduler
    void update();

    // true if a yaw beacon receiver is configured
    bool enabled() const { return Type(_type.get()) != Type::NONE; }

    // true if a configured backend is running
    bool healthy() const { return enabled() && _backend != nullptr; }

    // system time of the last accepted pulse, 0 if a pulse has never been received
    uint32_t last_pulse_ms() const { return _last_pulse_ms; }

    // called by backends when the receiver detects the beacon. pulse_us is the
    // system time of the detection edge in microseconds
    void handle_pulse(uint64_t pulse_us);

    // parameter accessors used by backends
    int8_t get_pin() const { return _pin.get(); }
    int8_t get_edge() const { return _edge.get(); }
    int16_t get_blank_ms() const { return _blank_ms.get(); }
    float get_beacon_yaw_deg() const { return _yaw_deg.get(); }

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_YawBeacon *_singleton;

    AP_Enum<Type> _type;    // receiver type, also acts as the enable parameter
    AP_Int8 _pin;           // GPIO pin the receiver output is connected to
    AP_Int8 _edge;          // signal edge marking detection (0:rising 1:falling)
    AP_Float _yaw_deg;      // vehicle heading when the receiver sees the beacon
    AP_Float _acc_deg;      // 1-sigma accuracy of a measurement
    AP_Int16 _blank_ms;     // pulses within this interval of the previous one are ignored
    AP_Float _delay_ms;     // fixed receiver latency subtracted from the pulse timestamp

    AP_YawBeacon_Backend *_backend;

    uint64_t _last_pulse_us;    // time of last accepted pulse
    uint32_t _last_pulse_ms;    // system time last pulse was accepted
    uint32_t _pulse_count;      // total accepted pulses
};

namespace AP {
    AP_YawBeacon *yawbeacon();
};

#endif  // AP_YAWBEACON_ENABLED
