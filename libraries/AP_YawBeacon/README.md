# AP_YawBeacon: absolute yaw from a stationary beacon

Driver for vehicles that rotate continuously in yaw (a few Hz) and carry a
narrow-field receiver — typically an IR receiver — that outputs a pulse each
time it sweeps past a stationary beacon. Each pulse is an absolute yaw
observation: at the instant of detection the vehicle's true heading equals a
configured value. The driver timestamps each pulse in the GPIO interrupt
handler and feeds it to EKF3 as an independent yaw angle measurement, giving
drift-free absolute yaw without a compass or GPS yaw.

## How it integrates with the EKF

`SourceYaw::YAWBEACON` (value 9) is a first-class EKF3 yaw source. Beacon
measurements travel through `AP_AHRS::writeEulerYawAngle` into
`NavEKF3::writeEulerYawAngle` (the generic external yaw-angle API, logged for
Replay as `REY3`) and share the buffered yaw-angle fusion path used by GPS
yaw:

- The first measurement after startup (or after a yaw source switch) performs
  a hard yaw alignment (`alignYawAngle`), instantly setting the filter's
  heading.
- Subsequent measurements are Kalman-fused against the quaternion states with
  measurement variance `radians(YBCN_ACC)^2` and the standard `EK3_YAW_I_GATE`
  innovation gate.
- Between pulses the EKF dead-reckons yaw on the gyro. At a couple of Hz of
  rotation the gyro drift accumulated over one revolution is negligible.
- While no pulses have been received yet the EKF keeps yaw variance bounded
  with synthetic yaw fusion, exactly as a GPS-yaw vehicle behaves before RTK
  lock. Yaw alignment completes on the first pulse, so the vehicle must
  be rotating (sweeping past the beacon) before position-dependent modes can
  be used.
- GPS yaw and beacon yaw share a measurement buffer, so writes are gated by
  the active yaw source: GPS yaw is ignored while `YAWBEACON` is selected and
  beacon pulses are ignored otherwise.

Timing accuracy dominates measurement quality on a fast-spinning vehicle: at
720 deg/sec, 1 ms of timestamp error is 0.72 degrees of yaw. Pulses are
timestamped in the interrupt handler (microsecond accuracy); fixed receiver
latency can be removed with `YBCN_DELAY` or simply folded into `YBCN_YAW` for
a constant spin direction.

## Configuration

| Parameter | Meaning |
|---|---|
| `YBCN_TYPE` | 0: disabled, 1: GPIO pin receiver, 10: SITL (simulation only). Reboot required. |
| `YBCN_PIN` | GPIO pin for the receiver output. Set a spare servo output's `SERVOx_FUNCTION` to -1 to use it as GPIO (e.g. AUX1..AUX6 = 50..55 on most boards). |
| `YBCN_YAW` | Vehicle true heading in degrees at the instant of detection: bearing to the beacon plus the receiver's mounting offset from the nose. |
| `YBCN_ACC` | 1-sigma measurement accuracy in degrees (default 5). Include the receiver's angular acceptance and timing jitter. |
| `YBCN_EDGE` | 0: rising edge, 1: falling edge. Demodulating IR receivers (TSOP series) are active-low: use 1. |
| `YBCN_BLANK` | Blanking time in ms after an accepted pulse (default 100). Rejects reflections and re-triggering; keep below the rotation period. |
| `YBCN_DELAY` | Fixed receiver latency in ms subtracted from each timestamp. |

EKF setup:

```
EK3_SRC1_YAW = 9   # YawBeacon
```

With the beacon as yaw source the magnetometer is not used for yaw; compasses
may be disabled entirely (`COMPASS_USE = 0`) on vehicles where the spin makes
them unusable.

The EKF accepts yaw measurements at most every 50 ms, so spin rates above
20 Hz drop measurements (harmless — one measurement per revolution is enough).

## Calibrating YBCN_YAW

Fly (or bench-spin) with logging enabled and compare `YBCN` pulse timestamps
against a reference heading (e.g. interpolate `ATT.Yaw` or SITL truth at
`YBCN.PT`). The mean difference is the correction to add to `YBCN_YAW`. Note
that receiver latency and the leading-edge-of-field-of-view geometry both
appear as a constant offset for a constant spin direction, so they calibrate
out together.

## SITL

`YBCN_TYPE = 10` synthesizes a pulse whenever the simulator's true yaw sweeps
through `YBCN_YAW`, with sub-sample timestamp interpolation. Example:

```
Tools/autotest/sim_vehicle.py -v ArduCopter --console \
    --add-param-file=<file with YBCN_TYPE=10, EK3_SRC1_YAW=9>
```

Arm in a non-position mode, take off, command a continuous yaw rate, and the
EKF yaw aligns on the first simulated pulse and tracks truth thereafter.

## Logging

Each accepted pulse logs a `YBCN` message: pulse system time (`PT`),
configured heading (`Yaw`), interval since the previous pulse (`DT`) and total
count (`Cnt`). EKF fusion health is visible in the standard `XKF`/`XKY`
innovation logging, and every measurement handed to the EKF is recorded as
`REY3` for Replay.

## Hardware notes

- Any receiver producing a clean digital edge works: demodulated IR (beacon
  modulated at 38 kHz driving a TSOP-style receiver is robust against
  sunlight), a photodiode with comparator, or even a hall sensor with a
  magnet for bench testing.
- Narrower receiver field of view gives better angular resolution; the
  blanking interval suppresses multiple triggers while the beacon is inside a
  wide field of view, keeping only the leading edge.
- Pre-arm: selecting `EK3_SRCn_YAW = 9` without `YBCN_TYPE` set fails arming
  with "EK3 sources require YawBeacon".
