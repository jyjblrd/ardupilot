#pragma once

#include "AP_IRBeaconYaw_config.h"

#if AP_IRBEACON_YAW_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

class AP_IRBeaconYaw {
public:
    AP_IRBeaconYaw();

    CLASS_NO_COPY(AP_IRBeaconYaw);

    enum class Edge : uint8_t {
        FALLING = 0,
        RISING = 1,
    };

    static AP_IRBeaconYaw *get_singleton() { return _singleton; }

    static const struct AP_Param::GroupInfo var_info[];

    // Attach the configured GPIO interrupt for pulse capture.
    void init();

    // True when a GPIO pin is configured for IR yaw capture.
    bool enabled() const { return _pin >= 0; }

    // True when the configured input interrupt was attached successfully.
    bool healthy() const { return enabled() && _interrupt_attached; }

    // Return the latest latched yaw sample from the receiver pulse.
    bool get_yaw_sample(float &yaw_rad, float &yaw_accuracy_rad, uint32_t &timestamp_ms, uint32_t &sequence);

private:
    void irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp_us);
    void update_min_interval();

    static AP_IRBeaconYaw *_singleton;

    AP_Int16 _pin;
    AP_Enum<Edge> _edge;
    AP_Float _yaw_deg;
    AP_Float _accuracy_deg;
    AP_Int16 _min_interval_ms;

    bool _interrupt_attached;
    volatile uint32_t _last_pulse_us;
    volatile uint32_t _pulse_sequence;
    volatile uint32_t _min_interval_us;
};

namespace AP {
    AP_IRBeaconYaw *irbeaconyaw();
};

#endif // AP_IRBEACON_YAW_ENABLED
