#include "AP_IRBeaconYaw.h"

#if AP_IRBEACON_YAW_ENABLED

#include "AP_IRBeaconYaw_Backend.h"
#include "AP_IRBeaconYaw_GPIO.h"
#include "AP_IRBeaconYaw_SITL.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>

AP_IRBeaconYaw *AP_IRBeaconYaw::_singleton;

const AP_Param::GroupInfo AP_IRBeaconYaw::var_info[] = {
    // Parameter indexes 1-5 are retained from the original GPIO-only driver.

    // @Param: PIN
    // @DisplayName: IR beacon yaw input pin
    // @Description: GPIO input pin connected to the IR receiver pulse output. Used when IRYAW_TYPE is GPIO.
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("PIN", 1, AP_IRBeaconYaw, _pin, -1),

    // @Param: EDGE
    // @DisplayName: IR beacon yaw pulse edge
    // @Description: Pulse edge that marks the configured yaw angle. Falling is normally used with active-low demodulating IR receivers.
    // @Values: 0:Falling,1:Rising
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EDGE", 2, AP_IRBeaconYaw, _edge, int8_t(Edge::FALLING)),

    // @Param: YAW
    // @DisplayName: IR beacon yaw angle
    // @Description: Vehicle yaw angle in degrees when the configured IR receiver pulse edge occurs.
    // @Units: deg
    // @Range: -180 360
    // @User: Advanced
    AP_GROUPINFO("YAW", 3, AP_IRBeaconYaw, _yaw_deg, 0.0f),

    // @Param: ACC
    // @DisplayName: IR beacon yaw accuracy
    // @Description: One-sigma accuracy of the yaw angle measurement produced by an IR receiver pulse.
    // @Units: deg
    // @Range: 1 45
    // @User: Advanced
    AP_GROUPINFO("ACC", 4, AP_IRBeaconYaw, _accuracy_deg, 5.0f),

    // @Param: MIN_MS
    // @DisplayName: IR beacon yaw minimum pulse interval
    // @Description: Minimum time between accepted receiver pulses. Use this to collapse pulse trains or chatter while the receiver crosses the beacon.
    // @Units: ms
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("MIN_MS", 5, AP_IRBeaconYaw, _min_interval_ms, 100),

    // @Param: TYPE
    // @DisplayName: IR beacon yaw receiver type
    // @Description: Type of IR beacon yaw receiver. GPIO captures a hardware pulse edge and SITL synthesizes a pulse as simulated yaw crosses IRYAW_YAW.
    // @Values: 0:None,1:GPIO,10:SITL
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 6, AP_IRBeaconYaw, _type, int8_t(Type::NONE), AP_PARAM_FLAG_ENABLE),

    // @Param: DELAY
    // @DisplayName: IR receiver latency
    // @Description: Fixed delay between pointing at the beacon and the receiver pulse edge. This delay is subtracted from the measurement timestamp. At 720 degrees per second each millisecond of uncorrected delay causes 0.72 degrees of yaw error.
    // @Units: ms
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("DELAY", 7, AP_IRBeaconYaw, _delay_ms, 0.0f),

    AP_GROUPEND
};

AP_IRBeaconYaw::AP_IRBeaconYaw() :
    _backend(nullptr),
    _last_pulse_us(0),
    _sample_time_ms(0),
    _pulse_sequence(0)
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_IRBeaconYaw must be singleton");
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_IRBeaconYaw::init()
{
    if (_backend != nullptr || !enabled()) {
        return;
    }

    switch (Type(_type.get())) {
    case Type::NONE:
        break;
    case Type::GPIO:
        _backend = NEW_NOTHROW AP_IRBeaconYaw_GPIO(*this);
        break;
#if AP_IRBEACON_YAW_SITL_ENABLED
    case Type::SITL:
        _backend = NEW_NOTHROW AP_IRBeaconYaw_SITL(*this);
        break;
#endif
    }

    if (_backend != nullptr) {
        _backend->init();
    }
}

void AP_IRBeaconYaw::update()
{
    if (_backend != nullptr) {
        _backend->update();
    }
}

bool AP_IRBeaconYaw::healthy() const
{
    return enabled() && _backend != nullptr && _backend->healthy();
}

bool AP_IRBeaconYaw::get_yaw_sample(float &yaw_rad, float &yaw_accuracy_rad, uint32_t &timestamp_ms, uint32_t &sequence)
{
    if (!healthy() || _pulse_sequence == 0) {
        return false;
    }

    timestamp_ms = _sample_time_ms;
    sequence = _pulse_sequence;
    yaw_rad = wrap_PI(radians(_yaw_deg.get()));
    yaw_accuracy_rad = radians(MAX(_accuracy_deg.get(), 1.0f));
    return true;
}

void AP_IRBeaconYaw::handle_pulse(uint64_t pulse_us)
{
    const uint64_t min_interval_us = uint64_t(MAX(_min_interval_ms.get(), 0)) * 1000ULL;
    if (_last_pulse_us != 0 &&
        (pulse_us <= _last_pulse_us || pulse_us - _last_pulse_us < min_interval_us)) {
        return;
    }

    const float dt = _last_pulse_us == 0 ? 0.0f : (pulse_us - _last_pulse_us) * 1.0e-6f;
    _last_pulse_us = pulse_us;
    const uint64_t sample_us = pulse_us - MIN(uint64_t(get_delay_us()), pulse_us);
    _sample_time_ms = uint32_t((sample_us + 500ULL) / 1000ULL);
    _pulse_sequence++;

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: IRYW
    // @Description: Accepted IR beacon yaw pulse
    // @Field: TimeUS: Time since system startup
    // @Field: PT: Raw receiver pulse edge time
    // @Field: MT: Latency-corrected measurement time
    // @Field: Yaw: Configured vehicle yaw at beacon detection
    // @Field: Acc: Configured one-sigma yaw accuracy
    // @Field: DT: Time since the previous accepted pulse
    // @Field: Cnt: Total accepted pulse count
    AP::logger().WriteStreaming("IRYW", "TimeUS,PT,MT,Yaw,Acc,DT,Cnt",
                                "sssdds-", "FFF000-", "QQQfffI",
                                AP_HAL::micros64(), pulse_us, sample_us,
                                _yaw_deg.get(), MAX(_accuracy_deg.get(), 1.0f), dt,
                                _pulse_sequence);
#endif
}

uint32_t AP_IRBeaconYaw::get_delay_us() const
{
    return uint32_t(constrain_float(_delay_ms.get(), 0.0f, 100.0f) * 1000.0f);
}

namespace AP {

AP_IRBeaconYaw *irbeaconyaw()
{
    return AP_IRBeaconYaw::get_singleton();
}

};

#endif // AP_IRBEACON_YAW_ENABLED
