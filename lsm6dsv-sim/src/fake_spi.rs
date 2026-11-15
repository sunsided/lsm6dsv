//! An `embedded-hal` SPI device backed by a [`Lsm6dsvModel`].

use crate::Lsm6dsvModel;
use core::cell::RefCell;
use embedded_hal::spi::{ErrorKind, ErrorType, Operation};

/// An SPI device (bus plus chip select) with a [`Lsm6dsvModel`] attached.
///
/// Every transaction is one chip-select cycle. The model does not distinguish between
/// 3-wire and 4-wire mode.
#[derive(Debug, Clone, Copy)]
pub struct FakeSpi<'a> {
    model: &'a RefCell<Lsm6dsvModel>,
}

impl<'a> FakeSpi<'a> {
    /// Creates a device with the specified model attached.
    pub fn new(model: &'a RefCell<Lsm6dsvModel>) -> Self {
        Self { model }
    }

    fn run(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), ErrorKind> {
        let mut model = self.model.borrow_mut();
        model.spi_select();
        for operation in operations {
            match operation {
                Operation::Write(bytes) => bytes.iter().for_each(|&b| {
                    model.spi_transfer(b);
                }),
                Operation::Read(buffer) => {
                    buffer.iter_mut().for_each(|b| *b = model.spi_transfer(0))
                }
                Operation::Transfer(read, write) => {
                    for i in 0..read.len().max(write.len()) {
                        let miso = model.spi_transfer(write.get(i).copied().unwrap_or(0));
                        if let Some(b) = read.get_mut(i) {
                            *b = miso;
                        }
                    }
                }
                Operation::TransferInPlace(buffer) => {
                    buffer.iter_mut().for_each(|b| *b = model.spi_transfer(*b))
                }
                Operation::DelayNs(_) => {}
            }
        }
        Ok(())
    }
}

impl ErrorType for FakeSpi<'_> {
    type Error = ErrorKind;
}

impl embedded_hal::spi::SpiDevice for FakeSpi<'_> {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        self.run(operations)
    }
}

impl embedded_hal_async::spi::SpiDevice for FakeSpi<'_> {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.run(operations)
    }
}
