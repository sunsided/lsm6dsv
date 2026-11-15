//! Main bank registers.

use crate::Bank;
use bitfield_struct::bitfield;

/// Main bank register addresses.
#[allow(non_camel_case_types)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum RegisterAddress {
    /// See [`WhoAmI`]. Read-only.
    WHO_AM_I = 0x0F,
}

impl RegisterAddress {
    /// Returns the address of a register.
    pub const fn addr(&self) -> u8 {
        *self as u8
    }
}

impl From<RegisterAddress> for u8 {
    fn from(value: RegisterAddress) -> Self {
        value.addr()
    }
}

/// [`WHO_AM_I`](RegisterAddress::WHO_AM_I) (0Fh): device identification.
///
/// Reads [`LSM6DSV_WHO_AM_I`](crate::LSM6DSV_WHO_AM_I) on both the LSM6DSV and the LSM6DSV16X.
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WhoAmI {
    /// The device identifier.
    #[bits(8, access = RO)]
    pub ident: u8,
}

readable_register!(WhoAmI, RegisterAddress::WHO_AM_I, Bank::Main);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prelude::*;

    #[test]
    fn who_am_i_addresses() {
        assert_eq!(
            <WhoAmI as I2CRegister<DeviceAddress7, RegisterAddress8, R1>>::REGISTER_ADDRESS,
            RegisterAddress8::new(0x0F)
        );
        assert_eq!(
            <WhoAmI as SPIRegister<RegisterAddress8, R1>>::REGISTER_ADDRESS,
            RegisterAddress8::new(0x0F)
        );
        assert_eq!(WhoAmI::BANK, Bank::Main);
        assert_eq!(WhoAmI::from_bits(0x70).ident(), crate::LSM6DSV_WHO_AM_I);
    }
}
