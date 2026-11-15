//! The auxiliary SPI2 interface for optical image stabilization (OIS).
//!
//! A second host (such as a camera module controller) can read OIS gyroscope and
//! accelerometer data over the auxiliary SPI bus (`OCS_Aux`, `SDO_Aux`, ...), independently
//! of the primary interface. [`Lsm6dsvOis`] drives that bus. It only reaches the
//! [`Spi2Register`] registers.
//!
//! The primary interface enables the OIS chain via `UI_CTRL1_OIS` (or hands control to
//! SPI2 with `FUNC_CFG_ACCESS.OIS_CTRL_FROM_UI` cleared).

use crate::data::i16x3;
use crate::interface::SpiInterface;
use crate::{AccelFullScale, Acceleration, AngularRate, GyroFullScale};
use lsm6dsv_registers::{
    Spi2Register, Spi2RegisterAddress, Spi2StatusRegOis, Spi2WhoAmI, WritableSpi2Register,
};

/// Driver for the auxiliary SPI2 (OIS) interface.
#[derive(Debug)]
pub struct Lsm6dsvOis<SPI> {
    iface: SpiInterface<SPI>,
}

impl<SPI> Lsm6dsvOis<SPI> {
    /// Creates a driver on the auxiliary SPI bus. Performs no bus access.
    pub fn new(spi: SPI) -> Self {
        Self {
            iface: SpiInterface::new(spi),
        }
    }

    /// Consumes the driver and returns the SPI device.
    pub fn release(self) -> SPI {
        self.iface.release()
    }
}

macro_rules! ois_methods {
    ($iface:path, [$($async:tt)?], [$($await:tt)*]) => {
        /// Reads the `SPI2_WHO_AM_I` register.
        pub $($async)? fn who_am_i(&mut self) -> Result<u8, <SpiInterface<SPI> as $iface>::Error> {
            Ok(self.read_register::<Spi2WhoAmI>() $($await)* ?.into())
        }

        /// Reads the OIS data-ready flags.
        pub $($async)? fn status(&mut self) -> Result<Spi2StatusRegOis, <SpiInterface<SPI> as $iface>::Error> {
            self.read_register() $($await)*
        }

        /// Reads the OIS gyroscope outputs at the specified full scale (`SPI2_CTRL2_OIS`).
        pub $($async)? fn read_angular_rate(&mut self, scale: GyroFullScale) -> Result<AngularRate, <SpiInterface<SPI> as $iface>::Error> {
            let mut bytes = [0; 6];
            <SpiInterface<SPI> as $iface>::read(&mut self.iface, Spi2RegisterAddress::SPI2_OUTX_L_G_OIS.addr(), &mut bytes) $($await)* ?;
            Ok(AngularRate { raw: i16x3(&bytes), scale })
        }

        /// Reads the OIS accelerometer outputs at the specified full scale (`SPI2_CTRL3_OIS`).
        pub $($async)? fn read_acceleration(&mut self, scale: AccelFullScale) -> Result<Acceleration, <SpiInterface<SPI> as $iface>::Error> {
            let mut bytes = [0; 6];
            <SpiInterface<SPI> as $iface>::read(&mut self.iface, Spi2RegisterAddress::SPI2_OUTX_L_A_OIS.addr(), &mut bytes) $($await)* ?;
            Ok(Acceleration { raw: i16x3(&bytes), scale })
        }

        /// Reads an SPI2 register.
        pub $($async)? fn read_register<R: Spi2Register>(&mut self) -> Result<R, <SpiInterface<SPI> as $iface>::Error> {
            let mut buffer = [0];
            <SpiInterface<SPI> as $iface>::read(&mut self.iface, R::ADDRESS, &mut buffer) $($await)* ?;
            Ok(R::from(buffer[0]))
        }

        /// Writes an SPI2 register.
        pub $($async)? fn write_register<R: WritableSpi2Register>(&mut self, register: R) -> Result<(), <SpiInterface<SPI> as $iface>::Error> {
            <SpiInterface<SPI> as $iface>::write(&mut self.iface, R::ADDRESS, &[register.into()]) $($await)*
        }
    };
}

impl<SPI> Lsm6dsvOis<SPI>
where
    SPI: embedded_hal::spi::SpiDevice,
{
    ois_methods!(crate::Interface, [], []);
}

/// Async methods, on `embedded-hal-async`.
#[cfg(feature = "async")]
#[cfg_attr(docsrs, doc(cfg(feature = "async")))]
pub mod asynch {
    use super::*;

    /// Async driver for the auxiliary SPI2 (OIS) interface.
    #[derive(Debug)]
    pub struct Lsm6dsvOis<SPI> {
        iface: SpiInterface<SPI>,
    }

    impl<SPI> Lsm6dsvOis<SPI> {
        /// Creates a driver on the auxiliary SPI bus. Performs no bus access.
        pub fn new(spi: SPI) -> Self {
            Self {
                iface: SpiInterface::new(spi),
            }
        }

        /// Consumes the driver and returns the SPI device.
        pub fn release(self) -> SPI {
            self.iface.release()
        }
    }

    impl<SPI> Lsm6dsvOis<SPI>
    where
        SPI: embedded_hal_async::spi::SpiDevice,
    {
        ois_methods!(crate::AsyncInterface, [async], [.await]);
    }
}
