# Changelog

All notable changes to this project will be documented in this file.
This project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- The full register map from datasheet DS13510: main, embedded functions and sensor hub banks,
  embedded advanced feature pages 0-2 (`AdvancedRegister`) and the auxiliary SPI2/OIS interface (`Spi2Register`).
- Typed field values: `DataRate`, `AccelMode`, `GyroMode`, `AccelFullScale`, `GyroFullScale`, `FifoMode`,
  `TemperatureBatchRate`, `TimestampDecimation`, `FifoTag`, `SflpDataRate`.
- The `lsm6dsv16x` feature for Qvar and machine learning core registers and bits.
- `Register::ADDRESS` and `Spi2Register::ADDRESS`, the address within the bank.
- Workspace skeleton with the `Register` / `WritableRegister` traits (I²C and SPI), the `Bank` enum and `WHO_AM_I`.
