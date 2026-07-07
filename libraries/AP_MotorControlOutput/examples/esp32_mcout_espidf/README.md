# ESP-IDF MCOUT Receiver

This ESP-IDF example receives ArduPilot `SERIALx_PROTOCOL=50` motor-control-output packets.
See the parent [Motor Control Output README](../../README.md) for the full ArduPilot setup, packet format, receiver requirements, and troubleshooting notes.

Default wiring for a LOLIN S2 Mini:

```text
ArduPilot UART TX -> ESP32-S2 GPIO16
ArduPilot GND     -> ESP32-S2 GND
```

ArduPilot parameters:

```text
SERIALx_PROTOCOL = 50
SERIALx_BAUD     = 921
MCOUT_RATE       = 400
```

Build and flash:

```sh
cd libraries/AP_MotorControlOutput/examples/esp32_mcout_espidf
source ~/.espressif/tools/activate_idf_v6.0.1.sh
idf.py set-target esp32s2
idf.py -p /dev/cu.usbmodem01 flash monitor
```

The example uses the ESP32-S2 native USB CDC console, so the USB serial device remains available
after the app boots.

The code intentionally does not drive motor outputs. Add actuator output only after validating
packet freshness, flags, and your custom mixer on the bench with props removed.
