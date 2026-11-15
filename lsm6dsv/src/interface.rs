//! Bus interfaces: I²C and SPI.

use lsm6dsv_registers::DEFAULT_DEVICE_ADDRESS;

/// The SPI read bit, set in the address byte of a read transfer.
const SPI_READ: u8 = 0x80;

/// Register access over a bus.
///
/// Implemented by [`I2cInterface`] and [`SpiInterface`]. Transfers of more than one byte
/// address consecutive registers while `CTRL3.IF_INC` is set (the default).
pub trait Interface {
    /// The bus error type.
    type Error;

    /// Reads consecutive registers, starting at `start`.
    fn read(&mut self, start: u8, buffer: &mut [u8]) -> Result<(), Self::Error>;

    /// Writes consecutive registers, starting at `start`.
    fn write(&mut self, start: u8, data: &[u8]) -> Result<(), Self::Error>;
}

/// Async register access over a bus.
///
/// Implemented by [`I2cInterface`] and [`SpiInterface`]. Transfers of more than one byte
/// address consecutive registers while `CTRL3.IF_INC` is set (the default).
#[cfg(feature = "async")]
#[cfg_attr(docsrs, doc(cfg(feature = "async")))]
// The trait is meant for single-threaded executors; no `Send` bound on the futures.
#[allow(async_fn_in_trait)]
pub trait AsyncInterface {
    /// The bus error type.
    type Error;

    /// Reads consecutive registers, starting at `start`.
    async fn read(&mut self, start: u8, buffer: &mut [u8]) -> Result<(), Self::Error>;

    /// Writes consecutive registers, starting at `start`.
    async fn write(&mut self, start: u8, data: &[u8]) -> Result<(), Self::Error>;
}

/// The LSM6DSV on an I²C bus.
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> I2cInterface<I2C> {
    /// Creates an interface for a device at [`DEFAULT_DEVICE_ADDRESS`].
    pub fn new(i2c: I2C) -> Self {
        Self::with_address(i2c, DEFAULT_DEVICE_ADDRESS)
    }

    /// Creates an interface for a device at the specified 7-bit address.
    pub fn with_address(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Returns the I²C bus.
    pub fn release(self) -> I2C {
        self.i2c
    }
}

impl<I2C> Interface for I2cInterface<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    type Error = I2C::Error;

    fn read(&mut self, start: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[start], buffer)
    }

    fn write(&mut self, start: u8, data: &[u8]) -> Result<(), Self::Error> {
        use embedded_hal::i2c::Operation;
        // Adjacent writes are sent as one transfer, without a repeated start.
        self.i2c.transaction(
            self.address,
            &mut [Operation::Write(&[start]), Operation::Write(data)],
        )
    }
}

#[cfg(feature = "async")]
impl<I2C> AsyncInterface for I2cInterface<I2C>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    type Error = I2C::Error;

    async fn read(&mut self, start: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[start], buffer).await
    }

    async fn write(&mut self, start: u8, data: &[u8]) -> Result<(), Self::Error> {
        use embedded_hal_async::i2c::Operation;
        self.i2c
            .transaction(
                self.address,
                &mut [Operation::Write(&[start]), Operation::Write(data)],
            )
            .await
    }
}

/// The LSM6DSV on an SPI bus (mode 0 or 3).
///
/// The device starts in 4-wire mode. For 3-wire mode, set
/// [`IfCfg::sim`](crate::registers::IfCfg) and use a HAL that supports half-duplex transfers.
/// When only SPI is used, I²C and MIPI I3C can be disabled with
/// [`IfCfg::i2c_i3c_disable`](crate::registers::IfCfg).
#[derive(Debug)]
pub struct SpiInterface<SPI> {
    spi: SPI,
}

impl<SPI> SpiInterface<SPI> {
    /// Creates an interface on the specified SPI device (bus plus chip select).
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Returns the SPI device.
    pub fn release(self) -> SPI {
        self.spi
    }
}

impl<SPI> Interface for SpiInterface<SPI>
where
    SPI: embedded_hal::spi::SpiDevice,
{
    type Error = SPI::Error;

    fn read(&mut self, start: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;
        self.spi.transaction(&mut [
            Operation::Write(&[start | SPI_READ]),
            Operation::Read(buffer),
        ])
    }

    fn write(&mut self, start: u8, data: &[u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;
        self.spi.transaction(&mut [
            Operation::Write(&[start & !SPI_READ]),
            Operation::Write(data),
        ])
    }
}

#[cfg(feature = "async")]
impl<SPI> AsyncInterface for SpiInterface<SPI>
where
    SPI: embedded_hal_async::spi::SpiDevice,
{
    type Error = SPI::Error;

    async fn read(&mut self, start: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal_async::spi::Operation;
        self.spi
            .transaction(&mut [
                Operation::Write(&[start | SPI_READ]),
                Operation::Read(buffer),
            ])
            .await
    }

    async fn write(&mut self, start: u8, data: &[u8]) -> Result<(), Self::Error> {
        use embedded_hal_async::spi::Operation;
        self.spi
            .transaction(&mut [
                Operation::Write(&[start & !SPI_READ]),
                Operation::Write(data),
            ])
            .await
    }
}
