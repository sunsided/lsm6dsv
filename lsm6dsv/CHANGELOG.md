# Changelog

All notable changes to this project will be documented in this file.
This project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `coordinate-frame` feature with the `SensorFrame<T>` alias (`EastNorthUp`) for the sensor axes, to convert
  readings into other frames with `coordinate-frame` 0.5.

## [0.3.0] - 2026-11-15

[0.3.0]: https://github.com/sunsided/lsm6dsv/releases/tag/v0.3.0

### Changed

- [**breaking**] `SelfTestError` is gone; the self-tests return `Error`, which has the same `Bus` and `Timeout` cases.
- [**breaking**] `f32_to_f16` is gone. It had no caller and rounded exact ties up instead of to even.
- `set_sensor_hub` panics for a slave count outside 1 to 4 instead of clamping it.

### Fixed

- After `write_configuration`, the driver reads the full scales from the device again. A `.ucf` sequence that
  changed them made `read_acceleration` and `read_angular_rate` scale with the old values.

## [0.2.0] - 2026-11-15

[0.2.0]: https://github.com/sunsided/lsm6dsv/releases/tag/v0.2.0

### Changed

- [**breaking**] Depends on `lsm6dsv-registers` 0.2, which moves to `hardware-registers` 0.4
  (re-exported as `lsm6dsv::registers`). The driver API is unchanged.

## [0.1.0] - 2026-11-15

[0.1.0]: https://github.com/sunsided/lsm6dsv/releases/tag/v0.1.0

### Added

- Blocking driver on `embedded-hal` 1.0 and an equivalent async driver (`async` feature), over I²C and SPI.
- Automatic register bank switching and embedded advanced feature page access.
- Configuration and scaled outputs, FIFO with tagged-word decoding, interrupts and motion events,
  embedded functions, FSM/MLC configuration sequences, sensor hub, and the SPI2 (OIS) interface.
- Qvar and machine learning core support (`lsm6dsv16x` feature).
- Filter configuration (`set_gyro_low_pass`, `set_accel_filter`) and self-test (`self_test_accel`, `self_test_gyro`)
  against the DS13510 limits.
- `FifoDecoder` restores 2x and 3x compressed accelerometer and gyroscope FIFO words.
