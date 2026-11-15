//! # STMicroelectronics LSM6DSV / LSM6DSV16X IMU driver
//!
//! > A platform-agnostic driver for the STMicroelectronics LSM6DSV and LSM6DSV16X 6-axis IMUs
//! > (accelerometer and gyroscope) over I²C or SPI.
//!
//! This driver was built using [`embedded-hal`] 1.0 and [`lsm6dsv-registers`]. Enable the
//! `async` feature for an equivalent driver on [`embedded-hal-async`] in the `asynch` module, and the
//! `lsm6dsv16x` feature for Qvar and machine learning core support.
//!
//! The driver takes the bus by value. To share the bus with other devices, pass a
//! bus handle such as `embedded_hal_bus::i2c::RefCellDevice`, or `&mut` to a bus.
//!
//! ## Example
//!
//! ```
//! # use core::cell::RefCell;
//! # use lsm6dsv_sim::{FakeDelay, FakeI2c, Lsm6dsvModel};
//! use lsm6dsv::{AccelFullScale, AccelMode, DataRate, GyroFullScale, GyroMode, Lsm6dsv};
//! # let model = RefCell::new(Lsm6dsvModel::new());
//! # let i2c = FakeI2c::new(&model);
//! # let mut delay = FakeDelay::new(&model);
//!
//! let mut imu = Lsm6dsv::new(i2c);
//! assert!(imu.identify()?);
//! imu.reset(&mut delay)?;
//!
//! imu.set_accel_full_scale(AccelFullScale::G4)?;
//! imu.set_gyro_full_scale(GyroFullScale::Dps500)?;
//! imu.set_accel_mode(DataRate::Hz120, AccelMode::HighPerformance)?;
//! imu.set_gyro_mode(DataRate::Hz120, GyroMode::HighPerformance)?;
//! # model.borrow_mut().set_accel([0, 0, 8197]);
//!
//! if imu.status()?.xlda() {
//!     let [x, y, z] = imu.read_acceleration()?.g();
//!     assert!((z - 1.0).abs() < 0.01);
//! }
//! # Ok::<(), lsm6dsv::Error<embedded_hal::i2c::ErrorKind>>(())
//! ```
//!
//! ## Register banks
//!
//! The device spreads its registers over a main bank, an embedded functions bank, a sensor
//! hub bank and indirectly addressed advanced feature pages. [`Lsm6dsv::read_register`],
//! [`Lsm6dsv::write_register`] and [`Lsm6dsv::modify_register`] take any register type of
//! [`registers`] and select its bank on their own; [`Lsm6dsv::read_advanced`] and
//! [`Lsm6dsv::write_advanced`] do the same for the advanced feature pages. Everything the
//! typed methods do not cover is reachable that way.
//!
//! ## FIFO
//!
//! [`Lsm6dsv::read_fifo`] reads tagged words; [`FifoWord::decode`] turns them into
//! [`FifoSample`]s. With FIFO compression enabled, decode with a [`FifoDecoder`], which keeps
//! the previous samples to restore compressed ones.
//!
//! [`embedded-hal`]: https://crates.io/crates/embedded-hal
//! [`embedded-hal-async`]: https://crates.io/crates/embedded-hal-async
//! [`lsm6dsv-registers`]: https://crates.io/crates/lsm6dsv-registers

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]
#![forbid(unsafe_code)]
// Enables the `doc_cfg` feature when the `docsrs` configuration attribute is defined.
#![cfg_attr(docsrs, feature(doc_cfg))]

#[cfg(feature = "async")]
#[cfg_attr(docsrs, doc(cfg(feature = "async")))]
pub mod asynch;
mod data;
mod driver;
mod fifo;
mod filter;
mod interface;
pub mod ois;
mod self_test;

pub use data::{
    Acceleration, AngularRate, QVAR_LSB_PER_MILLIVOLT, Quaternion, Qvar, SFLP_GRAVITY_MG_PER_LSB,
    STANDARD_GRAVITY, Temperature, Timestamp, f16_to_f32,
};
pub use driver::{Error, RESET_POLLS};
pub use fifo::{DecodedSamples, FifoDecoder, FifoSample, FifoStatus, FifoWord};
pub use filter::AccelFilter;
#[cfg(feature = "async")]
pub use interface::AsyncInterface;
pub use interface::{I2cInterface, Interface, SpiInterface};
pub use lsm6dsv_registers as registers;
pub use lsm6dsv_registers::{
    ALTERNATIVE_DEVICE_ADDRESS, AccelFullScale, AccelMode, Bank, DEFAULT_DEVICE_ADDRESS, DataRate,
    FifoMode, FifoTag, GyroFullScale, GyroMode, LSM6DSV_WHO_AM_I, SflpDataRate,
    TemperatureBatchRate, TimestampDecimation,
};
pub use lsm6dsv_registers::{AccelBandwidth, SelfTestMode};
pub use self_test::{
    ACCEL_SELF_TEST_MAX_MG, ACCEL_SELF_TEST_MIN_MG, GYRO_SELF_TEST_MAX_DPS, GYRO_SELF_TEST_MIN_DPS,
    SELF_TEST_SAMPLES, SelfTest,
};

pub use blocking::Lsm6dsv;

/// A vector in the sensor's own coordinate frame.
///
/// The LSM6DSV and LSM6DSV16X datasheets draw the accelerometer and gyroscope axes on the
/// same right-handed X/Y/Z set, with +Z pointing out of the top face of the package
/// (DS13476 Rev 5, Figure 4 "Pin connections", page 8; DS13510 Rev 4, Figure 5, page 9).
/// The sensor's X, Y and Z are therefore East, North and Up of [`EastNorthUp`](coordinate_frame::EastNorthUp).
///
/// Convert a reading with [`From<[T; 3]>`](From) and move it into your board frame with `.into()`:
///
/// ```
/// # use core::cell::RefCell;
/// # use lsm6dsv_sim::{FakeI2c, Lsm6dsvModel};
/// use coordinate_frame::NorthEastDown;
/// use lsm6dsv::{Lsm6dsv, SensorFrame};
/// # let model = RefCell::new(Lsm6dsvModel::new());
/// # let mut imu = Lsm6dsv::new(FakeI2c::new(&model));
///
/// let accel = SensorFrame::from(imu.read_acceleration()?.g());
/// let board: NorthEastDown<f32> = accel.into();
/// # Ok::<(), lsm6dsv::Error<embedded_hal::i2c::ErrorKind>>(())
/// ```
#[cfg(feature = "coordinate-frame")]
#[cfg_attr(docsrs, doc(cfg(feature = "coordinate-frame")))]
pub type SensorFrame<T> = coordinate_frame::EastNorthUp<T>;

/// The blocking driver.
mod blocking {
    crate::driver::define_driver!(crate::Interface, embedded_hal::delay::DelayNs, [], []);
}
