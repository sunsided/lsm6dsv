# Wokwi custom chip

Wraps the `lsm6dsv-sim` behavioral model as a [Wokwi](https://wokwi.com) custom chip.

- Pins: `VCC`, `GND`, `SDA`, `SCL`, `INT1` (data-ready, active high, as routed by `INT1_CTRL`).
- Controls: `accelX`, `accelY`, `accelZ` (mg) and `gyroX`, `gyroY`, `gyroZ` (dps). The chip converts
  them with the configured full scales and produces a new sample every 10 ms.

Build with `./build.sh`; it places `lsm6dsv.chip.wasm` next to `lsm6dsv.chip.json`.
See [`examples/esp32c3`](../../examples/esp32c3) for a firmware and scenario that use it.
