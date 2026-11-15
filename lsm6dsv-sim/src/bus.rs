//! [`fake_bus`] adapters for the [`Lsm6dsvModel`].

use crate::Lsm6dsvModel;
use core::cell::RefCell;
use fake_bus::{Clock, I2cTarget, SpiTarget};

/// An I²C bus with a single [`Lsm6dsvModel`] attached.
///
/// The model is shared through a [`RefCell`] so that tests can change the sensor data while
/// a driver owns the bus. For a bus shared with other models, use [`fake_bus::FakeI2c`] with
/// an array of targets.
pub type FakeI2c<'a> = fake_bus::FakeI2c<&'a RefCell<Lsm6dsvModel>>;

/// An SPI device (bus plus chip select) with a [`Lsm6dsvModel`] attached.
///
/// Every transaction is one chip-select cycle.
pub type FakeSpi<'a> = fake_bus::FakeSpi<'a, Lsm6dsvModel>;

/// A delay provider that advances the time of a [`Lsm6dsvModel`] instead of waiting.
pub type FakeDelay<'a> = fake_bus::FakeDelay<'a, Lsm6dsvModel>;

impl I2cTarget for Lsm6dsvModel {
    fn address(&self) -> u8 {
        Lsm6dsvModel::address(self)
    }

    fn start(&mut self, is_read: bool) -> bool {
        self.i2c_start(is_read)
    }

    fn write(&mut self, byte: u8) -> bool {
        self.i2c_write(byte)
    }

    fn read(&mut self) -> u8 {
        self.i2c_read()
    }
}

impl SpiTarget for Lsm6dsvModel {
    fn select(&mut self) {
        self.spi_select();
    }

    fn transfer(&mut self, mosi: u8) -> u8 {
        self.spi_transfer(mosi)
    }
}

impl Clock for Lsm6dsvModel {
    fn advance_ns(&mut self, ns: u64) {
        self.tick_ns(ns);
    }
}
