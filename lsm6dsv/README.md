# lsm6dsv

A platform-agnostic, `no_std` [`embedded-hal`](https://crates.io/crates/embedded-hal) 1.0 driver for the
STMicroelectronics LSM6DSV and LSM6DSV16X 6-axis IMUs, over I²C or SPI.

- Accelerometer, gyroscope, temperature and timestamp outputs, scaled to physical units
- Output data rates, operating modes, full scales, gyroscope LPF1 and accelerometer LPF2/high-pass filters
- Accelerometer and gyroscope self-test with the datasheet limits
- FIFO configuration, status and tagged-word decoding (accelerometer, gyroscope, temperature,
  timestamp, step counter, SFLP quaternion/gravity/bias, sensor hub), including compressed words
- Interrupt routing, wake-up, free-fall, tap, 6D/4D orientation and activity/inactivity
- Embedded functions: pedometer, tilt, significant motion, SFLP game rotation vector,
  FSM/MLC programs from `.ucf` sequences
- Sensor hub (I²C master for external sensors)
- Automatic register bank switching and access to the embedded advanced feature pages
- The auxiliary SPI2 (OIS) interface
- An equivalent async driver on `embedded-hal-async` with the `async` feature
- Qvar and machine learning core support with the `lsm6dsv16x` feature

Everything else is reachable through the typed registers of
[`lsm6dsv-registers`](../lsm6dsv-registers) with `read_register`, `write_register` and `modify_register`.

```rust,ignore
use lsm6dsv::{AccelFullScale, AccelMode, DataRate, Lsm6dsv};

let mut imu = Lsm6dsv::new(i2c);
imu.reset(&mut delay)?;
imu.set_accel_full_scale(AccelFullScale::G4)?;
imu.set_accel_mode(DataRate::Hz120, AccelMode::HighPerformance)?;
let [x, y, z] = imu.read_acceleration()?.g();
```

## Coordinate frames

With the `coordinate-frame` feature, `SensorFrame<T>` names the sensor's axes as a
[`coordinate-frame`](https://crates.io/crates/coordinate-frame) type. The datasheets draw a right-handed
X/Y/Z set with +Z out of the top face (DS13476 Rev 5, Figure 4; DS13510 Rev 4, Figure 5), so
`SensorFrame` is `EastNorthUp`. Convert readings into your board frame with `.into()`:

```rust,ignore
use coordinate_frame::NorthEastDown;
use lsm6dsv::SensorFrame;

let board: NorthEastDown<f32> = SensorFrame::from(imu.read_acceleration()?.g()).into();
```

## License

Licensed under any of the [EUPL-1.2](../LICENSE-EUPL), [MIT](../LICENSE-MIT) or [Apache-2.0](../LICENSE-APACHE) licenses, at your option.
