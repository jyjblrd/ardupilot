# Motor Control Output

Motor Control Output (`MCOUT`) publishes ArduPilot's normalized multicopter control state over a serial port for an external mixer or actuator controller. ArduPilot still runs the attitude controller, arming logic, interlock handling, spool-state tracking, and limit detection. The external device consumes the packet stream and applies the vehicle-specific mixing.

The feature is intended for bench-tested external mixing systems. The serial stream is binary packet data, not a text console.

## Configuration

Configure one ArduPilot serial port for the MotorControlOutput protocol:

```text
SERIALx_PROTOCOL = 50
SERIALx_BAUD     = 921
MCOUT_RATE       = 400
```

Replace `x` with the serial port connected to the external controller. For example, use `SERIAL1_PROTOCOL` and `SERIAL1_BAUD` for Serial1/Telem1.

`SERIALx_PROTOCOL=50` selects the `MotorControlOutput` protocol. When that protocol is selected, ArduPilot defaults the port to 921600 baud, disables flow control, and uses unbuffered writes. Setting `SERIALx_BAUD=921` makes that baud rate explicit in the parameter file.

`MCOUT_RATE` is the requested publish rate in Hz. It accepts values from 1 to 400 and defaults to 400. The actual output rate is limited by the vehicle scheduler loop rate.

### Pilot passthrough flight mode

ArduCopter flight mode `29` (`MCOUT_PASS`) sends the calibrated pilot input positions directly in the four primary command fields instead of sending attitude-controller output. Assign `29` to any `FLTMODE1` through `FLTMODE6` switch position; `FLTMODE_CH=5` uses RC channel 5 as the mode switch.

Roll, pitch, and yaw use the normal RC calibration and deadzone and are normalized to `-1..1`. Throttle uses the calibrated linear stick position from `0..1`; it does not use Copter's hover-throttle remapping. Arming, motor interlock, emergency stop, packet freshness, and all receiver-side safety requirements still apply. The mode also feeds these direct inputs into ArduCopter's normal motor path, so leave the FC motor functions unassigned when MCOUT is the only intended actuator output. Other flight modes continue to publish the normal stabilized control commands.

## Wiring

Connect the ArduPilot UART transmit pin to the external controller UART receive pin, and connect grounds:

```text
ArduPilot UART TX -> external controller UART RX
ArduPilot GND     -> external controller GND
```

MCOUT is currently one-way from ArduPilot to the external controller. ArduPilot does not read data from the MCOUT port.

The serial data line carries binary packets. If the ArduPilot UART is opened in a text terminal it will appear as unreadable characters. Use a separate console or telemetry link for readable status output.

## Packet Format

MCOUT sends one packed little-endian packet per update:

```c
struct ControlOutputPacket {
    uint16_t magic;                // 0xC04D
    uint8_t version;               // 1
    uint8_t length;                // 84
    uint16_t message_id;           // 1
    uint32_t sequence;
    uint64_t timestamp_usec;
    uint16_t flags;
    uint8_t spool_state;
    uint8_t desired_spool_state;
    float roll;
    float pitch;
    float yaw;
    float throttle;
    float roll_feedback;
    float pitch_feedback;
    float yaw_feedback;
    float roll_feedforward;
    float pitch_feedforward;
    float yaw_feedforward;
    float throttle_filtered;
    float throttle_out;
    float forward;
    float lateral;
    float dt;
    uint16_t crc;
};
```

The CRC is XMODEM CRC-16 over all packet bytes before the `crc` field. Receivers should validate `magic`, `version`, `length`, `message_id`, and `crc` before using a packet.

### Command Fields

`roll`, `pitch`, and `yaw` are normalized control commands formed from feedback plus feedforward terms. `throttle` is the filtered throttle command. These four primary command fields are set to zero unless ArduPilot considers actuator output enabled.

Output is enabled when motors are armed, the motor interlock is enabled, and emergency stop is not active. Receivers should still make their own output decision from packet freshness and flags before driving actuators.

The packet also carries the individual feedback and feedforward terms, `throttle_filtered`, `throttle_out`, `forward`, `lateral`, and ArduPilot motor update `dt` for monitoring or custom mixing.

### Flags

The `flags` field is a bitmask:

| Bit | Name | Meaning |
| - | - | - |
| 0 | `MOTORS_ARMED` | Motor library is armed |
| 1 | `SOFT_ARMED` | Vehicle is soft-armed |
| 2 | `INTERLOCK` | Motor interlock is enabled |
| 3 | `EMERGENCY_STOP` | Emergency stop is active |
| 4 | `SPOOLUP_BLOCK` | Motor spool-up is blocked |
| 5 | `MOTORS_INITIALIZED` | Motor library initialized successfully |
| 6 | `OUTPUT_ENABLED` | ArduPilot allowed nonzero primary commands in this packet |
| 7 | `LIMIT_ROLL` | Roll limit is active |
| 8 | `LIMIT_PITCH` | Pitch limit is active |
| 9 | `LIMIT_YAW` | Yaw limit is active |
| 10 | `LIMIT_THROTTLE_LOWER` | Lower throttle limit is active |
| 11 | `LIMIT_THROTTLE_UPPER` | Upper throttle limit is active |
| 12 | `THRUST_BOOST` | Thrust boost is active |
| 13 | `PILOT_PASSTHROUGH` | Primary commands are normalized pilot input positions from `MCOUT_PASS` mode |

### Spool States

`spool_state` uses:

| Value | State |
| - | - |
| 0 | `SHUT_DOWN` |
| 1 | `GROUND_IDLE` |
| 2 | `SPOOLING_UP` |
| 3 | `THROTTLE_UNLIMITED` |
| 4 | `SPOOLING_DOWN` |

`desired_spool_state` uses:

| Value | State |
| - | - |
| 0 | `SHUT_DOWN` |
| 1 | `GROUND_IDLE` |
| 2 | `THROTTLE_UNLIMITED` |

## Receiver Requirements

A receiver should:

- reject packets with bad framing, version, length, message ID, or CRC
- track `sequence` to detect dropped packets
- require fresh packets before driving outputs
- require the arming, soft-arm, interlock, and output-enabled flags before applying actuator commands
- immediately command a safe output state when packets become stale, emergency stop is set, or required flags are missing
- validate custom mixing on the bench with propellers removed before enabling real actuator output

The ESP-IDF example receiver in `examples/esp32_mcout_espidf` implements packet parsing, CRC validation, link statistics, freshness checks, and safe-output hooks. It intentionally does not drive motor outputs.

## ESP-IDF Receiver Example

The included ESP32-S2 example receives MCOUT on UART1 at 921600 baud and prints decoded packets on the board's USB CDC console.

For a LOLIN S2 Mini, the default wiring is:

```text
ArduPilot UART TX -> ESP32-S2 GPIO16
ArduPilot GND     -> ESP32-S2 GND
```

Build and flash the example:

```sh
cd libraries/AP_MotorControlOutput/examples/esp32_mcout_espidf
source ~/.espressif/tools/activate_idf_v6.0.1.sh
idf.py set-target esp32s2
idf.py -p /dev/cu.usbmodem01 flash monitor
```

Use the serial device that appears for your board in place of `/dev/cu.usbmodem01`.

Expected monitor output starts with a line like:

```text
Listening for ArduPilot MCOUT on UART1 RX GPIO16 at 921600 baud
```

If no valid ArduPilot packets are received, the example prints:

```text
waiting for valid MCOUT packets
```

After valid packets arrive, the monitor prints the receive rate, sequence rate, missed packet count, spool state, flags, command values, and feedback/feedforward terms.

## Troubleshooting

If the receiver prints `waiting for valid MCOUT packets`, check that the selected ArduPilot serial port has `SERIALx_PROTOCOL=50`, the baud rates match, grounds are connected, and the ArduPilot TX pin is wired to the receiver RX pin.

If the output looks like unreadable characters in a terminal, the binary MCOUT data stream is probably being viewed directly. Open the receiver's debug console instead.

If the receive rate is lower than `MCOUT_RATE`, check the vehicle scheduler loop rate and receiver-side UART handling. `MCOUT_RATE=400` requests 400 Hz but cannot exceed the scheduler rate.

If command fields remain zero while packets are valid, check the arming state, soft-arm state, interlock, and emergency stop flag. ArduPilot zeros the primary command fields unless output is enabled.
