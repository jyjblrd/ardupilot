#pragma once

#include "AP_IRBeaconYaw_config.h"

#if AP_IRBEACON_YAW_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

class AP_IRBeaconYaw_Backend;

class AP_IRBeaconYaw {
public:
    AP_IRBeaconYaw();

    CLASS_NO_COPY(AP_IRBeaconYaw);

    enum class Edge : uint8_t {
        FALLING = 0,
        RISING = 1,
    };

    enum class Type : uint8_t {
        NONE = 0,
        GPIO = 1,
#if AP_IRBEACON_YAW_SITL_ENABLED
        SITL = 10,
#endif
    };

    static AP_IRBeaconYaw *get_singleton() { return _singleton; }

    static const struct AP_Param::GroupInfo var_info[];

    // Allocate and initialise the configured receiver backend.
    void init();

    // Gather pulses from the receiver backend.
    void update();

    // True when a receiver backend is configured.
    bool enabled() const { return Type(_type.get()) != Type::NONE; }

    // True when the configured receiver backend is running.
    bool healthy() const;

    // Return the latest latched yaw sample from the receiver pulse.
    bool get_yaw_sample(float &yaw_rad, float &yaw_accuracy_rad, uint32_t &timestamp_ms, uint32_t &sequence);

    // Accept a raw receiver pulse timestamp from a backend.
    void handle_pulse(uint64_t pulse_us);

    int16_t get_pin() const { return _pin.get(); }
    Edge get_edge() const { return Edge(_edge.get()); }
    int16_t get_min_interval_ms() const { return _min_interval_ms.get(); }
    uint32_t get_delay_us() const;
    float get_beacon_yaw_deg() const { return _yaw_deg.get(); }

private:
    static AP_IRBeaconYaw *_singleton;

    AP_Enum<Type> _type;
    AP_Int16 _pin;
    AP_Enum<Edge> _edge;
    AP_Float _yaw_deg;
    AP_Float _accuracy_deg;
    AP_Int16 _min_interval_ms;
    AP_Float _delay_ms;

    AP_IRBeaconYaw_Backend *_backend;
    uint64_t _last_pulse_us;
    uint32_t _sample_time_ms;
    uint32_t _pulse_sequence;
};

namespace AP {
    AP_IRBeaconYaw *irbeaconyaw();
};

#endif // AP_IRBEACON_YAW_ENABLED
