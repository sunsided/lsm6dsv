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
    pub use crate::{
        AdvancedRegister, Bank, Register, Spi2Register, WritableAdvancedRegister, WritableRegister,
        WritableSpi2Register,
    };
    pub use hardware_registers::i2c::*;
    pub use hardware_registers::register_address::RegisterAddress8;
    pub use hardware_registers::sizes::R1;
    pub use hardware_registers::spi::{SPIRegister, SPIRegister8, WritableSPIRegister8};
    pub use hardware_registers::{FromBits, HardwareRegister, ToBits, WritableHardwareRegister};
}

macro_rules! bit_conversions {
    ($type:ident) => {
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

macro_rules! readable_register {
    ($type:ident, $addr:expr, $bank:expr) => {
        impl $crate::Register for $type {
            const BANK: $crate::Bank = $bank;
            const ADDRESS: u8 = ($addr).addr();
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

        bit_conversions!($type);
    };
}

macro_rules! writable_register {
    ($type:ident, $addr:expr, $bank:expr) => {
        readable_register!($type, $addr, $bank);
        impl $crate::prelude::WritableHardwareRegister<$crate::prelude::R1> for $type {}
        impl $crate::WritableRegister for $type {}
    };
}

macro_rules! readable_spi2_register {
    ($type:ident, $addr:expr) => {
        impl $crate::Spi2Register for $type {
            const ADDRESS: u8 = ($addr).addr();
        }
        impl $crate::prelude::HardwareRegister<$crate::prelude::R1> for $type {}

        impl $crate::prelude::SPIRegister<$crate::prelude::RegisterAddress8, $crate::prelude::R1>
            for $type
        {
            type Backing = u8;

            const REGISTER_ADDRESS: $crate::prelude::RegisterAddress8 =
                $crate::prelude::RegisterAddress8::new(($addr).addr());
        }

        bit_conversions!($type);
    };
}

macro_rules! writable_spi2_register {
    ($type:ident, $addr:expr) => {
        readable_spi2_register!($type, $addr);
        impl $crate::prelude::WritableHardwareRegister<$crate::prelude::R1> for $type {}
        impl $crate::WritableSpi2Register for $type {}
    };
}

macro_rules! readable_advanced_register {
    ($type:ident, $addr:expr) => {
        impl $crate::AdvancedRegister for $type {
            const PAGE: u8 = ($addr).page();
            const ADDRESS: u8 = ($addr).addr();
        }

        bit_conversions!($type);
    };
}

macro_rules! writable_advanced_register {
    ($type:ident, $addr:expr) => {
        readable_advanced_register!($type, $addr);
        impl $crate::WritableAdvancedRegister for $type {}
    };
}

mod advanced;
mod embedded;
mod main_bank;
mod sensor_hub;
mod spi2;
mod types;

pub use advanced::*;
pub use embedded::*;
pub use main_bank::*;
pub use sensor_hub::*;
pub use spi2::*;
pub use types::*;

/// The default I²C bus address, with the `SDO/SA0` pin pulled low.
pub const DEFAULT_DEVICE_ADDRESS: u8 = 0x6A;

/// The alternative I²C bus address, with the `SDO/SA0` pin pulled high.
pub const ALTERNATIVE_DEVICE_ADDRESS: u8 = 0x6B;

/// The value of the [`WhoAmI`] register for an LSM6DSV or LSM6DSV16X.
pub const LSM6DSV_WHO_AM_I: u8 = 0x70;

/// The value of the [`Spi2WhoAmI`] register.
pub const SPI2_WHO_AM_I: u8 = 0x70;

/// Temperature sensitivity, in LSB per °C.
///
/// A raw value of `0` corresponds to [`TEMPERATURE_OFFSET_CELSIUS`].
pub const TEMPERATURE_LSB_PER_CELSIUS: f32 = 256.0;

/// The temperature at which the temperature output reads `0`, in °C.
pub const TEMPERATURE_OFFSET_CELSIUS: f32 = 25.0;

/// Timestamp resolution, in microseconds per LSB (typical).
pub const TIMESTAMP_MICROSECONDS_PER_LSB: f32 = 21.75;

/// The register bank a register belongs to.
///
/// Banks other than [`Bank::Main`] are selected via [`FuncCfgAccess`].
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

impl Bank {
    /// Returns the [`FuncCfgAccess`] bank selection bits for this bank, as `(emb_func_reg_access, shub_reg_access)`.
    pub const fn selection(self) -> (bool, bool) {
        match self {
            Self::Main => (false, false),
            Self::EmbeddedFunctions => (true, false),
            Self::SensorHub => (false, true),
        }
    }
}

/// A register on the primary interface, addressable over I²C and SPI.
pub trait Register:
    prelude::I2CRegister8<prelude::DeviceAddress7> + prelude::SPIRegister8 + From<u8> + Into<u8>
{
    /// The bank this register lives in.
    const BANK: Bank;
    /// The register address within its bank.
    const ADDRESS: u8;
}

/// A writable register on the primary interface.
pub trait WritableRegister:
    prelude::WritableI2CRegister8<prelude::DeviceAddress7> + prelude::WritableSPIRegister8 + Register
{
}

/// A register of the auxiliary SPI2 (OIS) interface.
///
/// These registers are not reachable from the primary interface.
pub trait Spi2Register: prelude::SPIRegister8 + From<u8> + Into<u8> {
    /// The register address.
    const ADDRESS: u8;
}

/// A writable register of the auxiliary SPI2 (OIS) interface.
pub trait WritableSpi2Register: prelude::WritableSPIRegister8 + Spi2Register {}

/// An embedded advanced features register, accessed indirectly via the page registers
/// of the embedded functions bank.
pub trait AdvancedRegister:
    Copy + From<u8> + Into<u8> + prelude::ToBits<Target = u8> + prelude::FromBits<u8>
{
    /// The advanced features page.
    const PAGE: u8;
    /// The address within the page.
    const ADDRESS: u8;
}

/// A writable embedded advanced features register.
pub trait WritableAdvancedRegister: AdvancedRegister {}
