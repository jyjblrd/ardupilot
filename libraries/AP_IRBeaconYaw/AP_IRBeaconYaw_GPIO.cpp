#include "AP_IRBeaconYaw_GPIO.h"

#if AP_IRBEACON_YAW_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

void AP_IRBeaconYaw_GPIO::init()
{
    _blank_us = uint32_t(MAX(_frontend.get_min_interval_ms(), 0)) * 1000U;

    const int16_t pin = _frontend.get_pin();
    if (pin < 0 || pin > UINT8_MAX || !hal.gpio->valid_pin(uint8_t(pin))) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IRYAW: invalid GPIO %d", int(pin));
        return;
    }

    const AP_HAL::GPIO::INTERRUPT_TRIGGER_TYPE trigger =
        _frontend.get_edge() == AP_IRBeaconYaw::Edge::RISING ?
        AP_HAL::GPIO::INTERRUPT_RISING : AP_HAL::GPIO::INTERRUPT_FALLING;

    hal.gpio->pinMode(uint8_t(pin), HAL_GPIO_INPUT);
    _interrupt_attached = hal.gpio->attach_interrupt(
        uint8_t(pin),
        FUNCTOR_BIND_MEMBER(&AP_IRBeaconYaw_GPIO::irq_handler, void, uint8_t, bool, uint32_t),
        trigger);

    if (!_interrupt_attached) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IRYAW: failed to attach GPIO %d", int(pin));
    }
}

void AP_IRBeaconYaw_GPIO::update()
{
    _blank_us = uint32_t(MAX(_frontend.get_min_interval_ms(), 0)) * 1000U;

    void *irq_state = hal.scheduler->disable_interrupts_save();
    const uint32_t count = _pulse_count;
    const uint32_t pulse_us32 = _pulse_us;
    hal.scheduler->restore_interrupts(irq_state);

    if (count == _last_count) {
        return;
    }
    _last_count = count;

    // Extend the recent 32-bit interrupt time using the current 64-bit clock.
    const uint64_t now_us = AP_HAL::micros64();
    const uint64_t pulse_us = now_us - uint32_t(uint32_t(now_us) - pulse_us32);
    _frontend.handle_pulse(pulse_us);
}

void AP_IRBeaconYaw_GPIO::irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp_us)
{
    (void)pin;
    (void)pin_state;

    if (_pulse_count != 0 && timestamp_us - _pulse_us < _blank_us) {
        return;
    }

    _pulse_us = timestamp_us;
    _pulse_count++;
}

#endif // AP_IRBEACON_YAW_ENABLED
