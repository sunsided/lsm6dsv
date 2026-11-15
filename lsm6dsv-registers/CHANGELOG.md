# Changelog

All notable changes to this project will be documented in this file.
This project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed

- LSM6DSV: `EmbFuncEnB::pedo_adv_en` (bit 4) and `PedoCmdReg::fp_rejection_en` (bit 2), as in DS13476;
  ST's LSM6DSV C header leaves both out. `fp_rejection_en` is now available on both devices.
- `ExtFormat::ext_format_sel` is bit 2, as in DS13510 table 427 (it was bit 1, as in ST's C headers).

### Added

- The full register map from datasheet DS13510: main, embedded functions and sensor hub banks,
  embedded advanced feature pages 0-2 (`AdvancedRegister`) and the auxiliary SPI2/OIS interface (`Spi2Register`).
- Typed field values: `DataRate`, `AccelMode`, `GyroMode`, `AccelFullScale`, `GyroFullScale`, `FifoMode`,
  `TemperatureBatchRate`, `TimestampDecimation`, `FifoTag`, `SflpDataRate`, `AccelBandwidth`, `SelfTestMode`.
- The `lsm6dsv16x` feature for Qvar and machine learning core registers and bits.
- `Register::ADDRESS` and `Spi2Register::ADDRESS`, the address within the bank.
- Workspace skeleton with the `Register` / `WritableRegister` traits (I²C and SPI), the `Bank` enum and `WHO_AM_I`.
