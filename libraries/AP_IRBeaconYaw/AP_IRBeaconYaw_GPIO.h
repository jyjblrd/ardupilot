#pragma once

#include "AP_IRBeaconYaw_config.h"

#if AP_IRBEACON_YAW_ENABLED

#include "AP_IRBeaconYaw_Backend.h"

// GPIO receiver backend. The interrupt timestamps the edge and applies
// blanking before reflections can overwrite the real pulse.
class AP_IRBeaconYaw_GPIO : public AP_IRBeaconYaw_Backend {
public:
    using AP_IRBeaconYaw_Backend::AP_IRBeaconYaw_Backend;

    void init() override;
    void update() override;
    bool healthy() const override { return _interrupt_attached; }

private:
    void irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp_us);

    bool _interrupt_attached = false;
    volatile uint32_t _pulse_us = 0;
    volatile uint32_t _blank_us = 0;
    volatile uint32_t _pulse_count = 0;
    uint32_t _last_count = 0;
};

#endif // AP_IRBEACON_YAW_ENABLED
