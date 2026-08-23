# IR Beacon Yaw

`AP_IRBeaconYaw` turns a pulse from a fixed IR beacon receiver into an intermittent absolute-yaw measurement for EKF3. Gyro integration carries yaw between pulses; each pulse corrects accumulated yaw drift without replacing the normal accelerometer, GPS, barometer, or optical-flow data paths.

Configure a hardware GPIO receiver and select it as the EKF yaw source:

```text
IRYAW_TYPE     = 1   # GPIO
IRYAW_PIN      = <GPIO input pin>
IRYAW_EDGE     = 0   # Falling, or 1 for Rising
IRYAW_YAW      = <vehicle yaw at pulse, deg>
IRYAW_ACC      = <1-sigma yaw accuracy, deg>
IRYAW_MIN_MS   = 100
IRYAW_DELAY    = <fixed receiver latency, ms>
EK3_SRC1_YAW   = 9   # IRBeacon
```

`IRYAW_MIN_MS` rejects repeated edges while the receiver crosses the beacon. It must remain shorter than the fastest expected rotation period. `IRYAW_DELAY` is subtracted from the interrupt timestamp; at 720 degrees per second, 1 ms of uncorrected delay creates 0.72 degrees of fixed yaw error.

The GPIO interrupt timestamps and blanks pulses before the 100 Hz backend task transfers them to the frontend. Each EKF3 core consumes the sample once through a dedicated IR-beacon buffer and fusion method. The `IRYW` log records raw and corrected pulse times, configured yaw and accuracy, pulse interval, and pulse count.

While the receiver is enabled, `NAMED_VALUE_FLOAT.IRYW_CNT` publishes the accepted pulse count over MAVLink once per second for live inspection without onboard logging.

Demodulating IR receivers commonly have active-low outputs, so use the falling edge. A servo output can be used as a GPIO when the board supports it by setting its `SERVOx_FUNCTION` to `-1`; use the board's documented GPIO number for `IRYAW_PIN`. The receiver should provide a clean digital edge, and its field of view and timing jitter should be included in `IRYAW_ACC`.

Calibrate `IRYAW_YAW` by interpolating a trusted heading at each `IRYW.PT` timestamp. A fixed mounting offset, field-of-view leading edge, and any residual receiver latency appear as a constant yaw offset for one spin direction. Reversing spin direction changes the sign of timing-related error, so bidirectional operation requires an accurate `IRYAW_DELAY` rather than folding all latency into `IRYAW_YAW`.

Before the first pulse, yaw has no absolute beacon reference. Take off and begin rotation in a mode that does not require position-aided yaw alignment. Between pulses the EKF propagates attitude with the gyros; accelerometers constrain roll and pitch but do not provide absolute yaw. GPS position/velocity, barometer, optical flow, and other selected sources continue through their normal fusion paths. A nearby beacon also only has a constant configured yaw if vehicle translation does not materially change its bearing.

For SITL, set `IRYAW_TYPE=10`. The simulator generates a pulse when true yaw crosses `IRYAW_YAW`, including the configured receiver delay, so the same timestamp-correction path is exercised by the Copter autotest.
