#include "AP_IRBeaconYaw.h"

#if AP_IRBEACON_YAW_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_IRBeaconYaw *AP_IRBeaconYaw::_singleton;

const AP_Param::GroupInfo AP_IRBeaconYaw::var_info[] = {
    // @Param: PIN
    // @DisplayName: IR beacon yaw input pin
    // @Description: GPIO input pin connected to the IR receiver pulse output. Set to -1 to disable IR beacon yaw.
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

    AP_GROUPEND
};

AP_IRBeaconYaw::AP_IRBeaconYaw() :
    _interrupt_attached(false),
    _last_pulse_us(0),
    _pulse_sequence(0),
    _min_interval_us(0)
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_IRBeaconYaw must be singleton");
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_IRBeaconYaw::init()
{
    update_min_interval();

    if (!enabled()) {
        return;
    }

    const int16_t pin = _pin.get();
    if ((pin < 0) || (pin > UINT8_MAX) || !hal.gpio->valid_pin(uint8_t(pin))) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IRYAW: invalid GPIO %d", int(pin));
        return;
    }

    const AP_HAL::GPIO::INTERRUPT_TRIGGER_TYPE trigger =
        (Edge(_edge.get()) == Edge::RISING) ? AP_HAL::GPIO::INTERRUPT_RISING : AP_HAL::GPIO::INTERRUPT_FALLING;

    hal.gpio->pinMode(uint8_t(pin), HAL_GPIO_INPUT);
    _interrupt_attached = hal.gpio->attach_interrupt(
        uint8_t(pin),
        FUNCTOR_BIND_MEMBER(&AP_IRBeaconYaw::irq_handler, void, uint8_t, bool, uint32_t),
        trigger);

    if (!_interrupt_attached) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IRYAW: failed to attach GPIO %d", int(pin));
    }
}

bool AP_IRBeaconYaw::get_yaw_sample(float &yaw_rad, float &yaw_accuracy_rad, uint32_t &timestamp_ms, uint32_t &sequence)
{
    update_min_interval();

    if (!_interrupt_attached) {
        return false;
    }

    uint32_t sequence_before;
    uint32_t timestamp_us;
    do {
        sequence_before = _pulse_sequence;
        timestamp_us = _last_pulse_us;
        sequence = _pulse_sequence;
    } while (sequence_before != sequence);

    if (sequence == 0) {
        return false;
    }

    timestamp_ms = timestamp_us / 1000U;
    yaw_rad = wrap_PI(radians(_yaw_deg.get()));
    yaw_accuracy_rad = radians(MAX(_accuracy_deg.get(), 1.0f));
    return true;
}

void AP_IRBeaconYaw::irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp_us)
{
    (void)pin;
    (void)pin_state;

    const uint32_t min_interval_us = _min_interval_us;
    if ((min_interval_us != 0) && (timestamp_us - _last_pulse_us < min_interval_us)) {
        return;
    }

    _last_pulse_us = timestamp_us;
    _pulse_sequence++;
}

void AP_IRBeaconYaw::update_min_interval()
{
    _min_interval_us = uint32_t(MAX(_min_interval_ms.get(), 0)) * 1000U;
}

namespace AP {

AP_IRBeaconYaw *irbeaconyaw()
{
    return AP_IRBeaconYaw::get_singleton();
}

};

#endif // AP_IRBEACON_YAW_ENABLED
