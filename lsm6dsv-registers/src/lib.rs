//! # STMicroelectronics LSM6DSV / LSM6DSV16X IMU Registers
//!
//! This crate provides a typed map of the LSM6DSV and LSM6DSV16X registers.
//! Every register can be addressed over I²C and SPI.
//!
//! The device has several register banks, which are selected via `FUNC_CFG_ACCESS`.
//! Each register states its bank in [`Register::BANK`].
//! Registers and fields that exist only on the LSM6DSV16X are behind the `lsm6dsv16x` feature.

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]
#![forbid(unsafe_code)]
// Enables the `doc_cfg` feature when the `docsrs` configuration attribute is defined.
#![cfg_attr(docsrs, feature(doc_cfg))]

/// Exports commonly used traits.
pub mod prelude {
    pub use crate::{Bank, Register, WritableRegister};
    pub use hardware_registers::i2c::*;
    pub use hardware_registers::register_address::RegisterAddress8;
    pub use hardware_registers::sizes::R1;
    pub use hardware_registers::spi::{SPIRegister, SPIRegister8, WritableSPIRegister8};
    pub use hardware_registers::{FromBits, HardwareRegister, ToBits, WritableHardwareRegister};
}

macro_rules! readable_register {
    ($type:ident, $addr:expr, $bank:expr) => {
        impl $crate::Register for $type {
            const BANK: $crate::Bank = $bank;
        }
        impl $crate::prelude::HardwareRegister<$crate::prelude::R1> for $type {}

        impl
            $crate::prelude::I2CRegister<
                $crate::prelude::DeviceAddress7,
                $crate::prelude::RegisterAddress8,
                $crate::prelude::R1,
            > for $type
        {
            type Backing = u8;

            const DEFAULT_DEVICE_ADDRESS: $crate::prelude::DeviceAddress7 =
                $crate::prelude::DeviceAddress7::new($crate::DEFAULT_DEVICE_ADDRESS);
            const REGISTER_ADDRESS: $crate::prelude::RegisterAddress8 =
                $crate::prelude::RegisterAddress8::new(($addr).addr());
        }

        impl $crate::prelude::SPIRegister<$crate::prelude::RegisterAddress8, $crate::prelude::R1>
            for $type
        {
            type Backing = u8;

            const REGISTER_ADDRESS: $crate::prelude::RegisterAddress8 =
                $crate::prelude::RegisterAddress8::new(($addr).addr());
        }

        impl $crate::prelude::ToBits for $type {
            type Target = u8;

            #[inline]
            fn to_bits(&self) -> Self::Target {
                (*self).into()
            }
        }

        impl $crate::prelude::FromBits<u8> for $type {
            #[inline]
            fn from_bits(value: u8) -> Self {
                value.into()
            }

            #[inline]
            fn from_bits_ref(value: &u8) -> Self {
                (*value).into()
            }
        }
    };
}

#[allow(unused_macros)] // used once the first writable register lands
macro_rules! writable_register {
    ($type:ident, $addr:expr, $bank:expr) => {
        readable_register!($type, $addr, $bank);
        impl $crate::prelude::WritableHardwareRegister<$crate::prelude::R1> for $type {}
        impl $crate::WritableRegister for $type {}
    };
}

mod registers;

pub use registers::*;

/// The default I²C bus address, with the `SDO/SA0` pin pulled low.
pub const DEFAULT_DEVICE_ADDRESS: u8 = 0x6A;

/// The alternative I²C bus address, with the `SDO/SA0` pin pulled high.
pub const ALTERNATIVE_DEVICE_ADDRESS: u8 = 0x6B;

/// The value of the [`WhoAmI`] register for an LSM6DSV or LSM6DSV16X.
pub const LSM6DSV_WHO_AM_I: u8 = 0x70;

/// The register bank a register belongs to.
///
/// Banks other than [`Bank::Main`] are selected via `FUNC_CFG_ACCESS`.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Bank {
    /// The main (user interface) bank.
    Main,
    /// The embedded functions bank (`EMB_FUNC_REG_ACCESS = 1`).
    EmbeddedFunctions,
    /// The sensor hub bank (`SHUB_REG_ACCESS = 1`).
    SensorHub,
}

/// An IMU register, addressable over I²C and SPI.
pub trait Register:
    prelude::I2CRegister8<prelude::DeviceAddress7> + prelude::SPIRegister8 + From<u8> + Into<u8>
{
    /// The bank this register lives in.
    const BANK: Bank;
}

/// A writable IMU register.
pub trait WritableRegister:
    prelude::WritableI2CRegister8<prelude::DeviceAddress7> + prelude::WritableSPIRegister8 + Register
{
}
