#pragma once

#include <AP_SerialManager/AP_SerialManager_config.h>

#ifndef AP_MOTOR_CONTROL_OUTPUT_ENABLED
#define AP_MOTOR_CONTROL_OUTPUT_ENABLED (AP_SERIALMANAGER_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif
