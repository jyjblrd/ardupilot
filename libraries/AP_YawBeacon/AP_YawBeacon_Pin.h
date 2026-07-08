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

#include "AP_YawBeacon_Backend.h"

/*
  backend for a receiver connected to a GPIO pin. The receiver's detection
  edge is timestamped in the pin interrupt handler for accuracy; on a vehicle
  spinning at 720 deg/sec each millisecond of timing error is 0.72 degrees of
  yaw error.
 */
class AP_YawBeacon_Pin : public AP_YawBeacon_Backend {
public:
    using AP_YawBeacon_Backend::AP_YawBeacon_Backend;

    void update() override;

private:
    void irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp);

    int8_t last_pin = -1;       // last pin number attached, checked vs PIN parameter
    int8_t last_edge = -1;      // last edge attached, checked vs EDGE parameter
    bool interrupt_attached;    // true if an interrupt is attached to last_pin

    // state shared with the interrupt handler. Not volatile; consistency comes
    // from reading it with interrupts disabled in update()
    struct IrqState {
        uint32_t pulse_us;      // timestamp of the latest accepted pulse (32-bit micros)
        uint32_t blank_us;      // pulses closer together than this are ignored
        uint32_t count;         // number of accepted pulses
    } irq_state;
    uint32_t last_count;        // pulse count already consumed by update()
};

#endif  // AP_YAWBEACON_ENABLED
