#pragma once

#include "AP_IRBeaconYaw_config.h"

#if AP_IRBEACON_YAW_SITL_ENABLED

#include "AP_IRBeaconYaw_Backend.h"

// SITL synthesizes a receiver edge when true yaw crosses the configured yaw.
class AP_IRBeaconYaw_SITL : public AP_IRBeaconYaw_Backend {
public:
    using AP_IRBeaconYaw_Backend::AP_IRBeaconYaw_Backend;

    void init() override;
    void update() override;
    bool healthy() const override { return _healthy; }

private:
    bool _healthy = false;
    bool _have_previous = false;
    float _previous_yaw_deg = 0.0f;
    uint64_t _previous_time_us = 0;
    uint64_t _pending_pulse_us = 0;
};

#endif // AP_IRBEACON_YAW_SITL_ENABLED
