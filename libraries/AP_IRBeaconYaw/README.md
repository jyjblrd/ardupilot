# IR Beacon Yaw

`AP_IRBeaconYaw` turns a GPIO pulse from a fixed IR beacon receiver into an absolute yaw measurement for EKF3.

Configure the receiver pin and yaw source:

```text
IRYAW_PIN      = <GPIO input pin>
IRYAW_EDGE     = 0   # Falling, or 1 for Rising
IRYAW_YAW      = <vehicle yaw at pulse, deg>
IRYAW_ACC      = <1-sigma yaw accuracy, deg>
IRYAW_MIN_MS   = 100
EK3_SRC1_YAW   = 9   # IRBeacon
```

`IRYAW_MIN_MS` rejects repeated pulse edges while the receiver crosses the beacon. The EKF timestamps each accepted edge at interrupt time and fuses it as an intermittent absolute yaw angle.
