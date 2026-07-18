#pragma once

#include "AP_IRBeaconYaw_config.h"

#if AP_IRBEACON_YAW_ENABLED

#include "AP_IRBeaconYaw.h"

class AP_IRBeaconYaw_Backend {
public:
    explicit AP_IRBeaconYaw_Backend(AP_IRBeaconYaw &frontend) :
        _frontend(frontend)
    {}

    virtual ~AP_IRBeaconYaw_Backend() = default;

    // Initialise the receiver and report pulses from update().
    virtual void init() = 0;
    virtual void update() = 0;
    virtual bool healthy() const = 0;

protected:
    AP_IRBeaconYaw &_frontend;
};

#endif // AP_IRBEACON_YAW_ENABLED
