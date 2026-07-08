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

#include "AP_YawBeacon_Pin.h"

#if AP_YAWBEACON_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/GPIO.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

void AP_YawBeacon_Pin::irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp)
{
    // apply the blanking interval here so a reflection cannot overwrite the
    // true detection edge before update() consumes it
    if (irq_state.count != 0 && (timestamp - irq_state.pulse_us) < irq_state.blank_us) {
        return;
    }
    irq_state.pulse_us = timestamp;
    irq_state.count++;
}

void AP_YawBeacon_Pin::update(void)
{
    // refresh the blanking interval used by the interrupt handler. A single
    // word write does not need interrupt protection
    irq_state.blank_us = (uint32_t)MAX(_frontend.get_blank_ms(), 0) * 1000U;

    const int8_t pin = _frontend.get_pin();
    const int8_t edge = _frontend.get_edge();
    if (last_pin != pin || last_edge != edge) {
        // detach from the previous pin
        if (interrupt_attached) {
            // ignore failure as the user may be stuck
            IGNORE_RETURN(hal.gpio->detach_interrupt(last_pin));
            interrupt_attached = false;
        }
        last_pin = pin;
        last_edge = edge;
        // attach to the new pin
        if (pin > 0) {
            hal.gpio->pinMode(pin, HAL_GPIO_INPUT);
            if (hal.gpio->attach_interrupt(
                    pin,
                    FUNCTOR_BIND_MEMBER(&AP_YawBeacon_Pin::irq_handler, void, uint8_t, bool, uint32_t),
                    edge == 1 ? AP_HAL::GPIO::INTERRUPT_FALLING : AP_HAL::GPIO::INTERRUPT_RISING)) {
                interrupt_attached = true;
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "YawBeacon: failed to attach to pin %d", pin);
            }
        }
    }

    // consume pulses recorded by the interrupt handler
    void *irqstate = hal.scheduler->disable_interrupts_save();
    const uint32_t count = irq_state.count;
    const uint32_t pulse_us32 = irq_state.pulse_us;
    hal.scheduler->restore_interrupts(irqstate);

    if (count == last_count) {
        return;
    }
    last_count = count;

    // extend the 32-bit interrupt timestamp to 64 bits. The pulse is always in
    // the recent past so the difference to now is small and cannot have wrapped
    const uint64_t now_us = AP_HAL::micros64();
    const uint64_t pulse_us = now_us - (uint32_t)((uint32_t)now_us - pulse_us32);
    _frontend.handle_pulse(pulse_us);
}

#endif  // AP_YAWBEACON_ENABLED
