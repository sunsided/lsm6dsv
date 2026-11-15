# Changelog

All notable changes to this project will be documented in this file.
This project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Blocking driver on `embedded-hal` 1.0 and an equivalent async driver (`async` feature), over I²C and SPI.
- Automatic register bank switching and embedded advanced feature page access.
- Configuration and scaled outputs, FIFO with tagged-word decoding, interrupts and motion events,
  embedded functions, FSM/MLC configuration sequences, sensor hub, and the SPI2 (OIS) interface.
- Qvar and machine learning core support (`lsm6dsv16x` feature).
- Filter configuration (`set_gyro_low_pass`, `set_accel_filter`) and self-test (`self_test_accel`, `self_test_gyro`)
  against the DS13510 limits.
