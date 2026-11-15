//! An `embedded-hal` I²C bus backed by a [`Lsm6dsvModel`].

use crate::Lsm6dsvModel;
use core::cell::RefCell;
use embedded_hal::i2c::{ErrorKind, ErrorType, NoAcknowledgeSource, Operation, SevenBitAddress};

/// An I²C bus with a single [`Lsm6dsvModel`] attached.
///
/// The model is shared through a [`RefCell`] so that tests can change the sensor data while a
/// driver owns the bus.
#[derive(Debug, Clone, Copy)]
pub struct FakeI2c<'a> {
    model: &'a RefCell<Lsm6dsvModel>,
}

impl<'a> FakeI2c<'a> {
    /// Creates a bus with the specified model attached.
    pub fn new(model: &'a RefCell<Lsm6dsvModel>) -> Self {
        Self { model }
    }

    fn run(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), ErrorKind> {
        let mut model = self.model.borrow_mut();
        if address != model.address() {
            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
        }

        // Adjacent operations of the same type form one transfer, without a repeated start.
        let mut previous_is_read = None;
        for operation in operations {
            let is_read = matches!(operation, Operation::Read(_));
            let restart = previous_is_read != Some(is_read);
            previous_is_read = Some(is_read);
            match operation {
                Operation::Write(bytes) => {
                    if restart && !model.i2c_start(false) {
                        return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
                    }

                    if !bytes.iter().all(|&b| model.i2c_write(b)) {
                        return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
                    }
                }

                Operation::Read(buffer) => {
                    if restart && !model.i2c_start(true) {
                        return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
                    }

                    buffer.iter_mut().for_each(|b| *b = model.i2c_read());
                }
            }
        }

        Ok(())
    }
}

impl ErrorType for FakeI2c<'_> {
    type Error = ErrorKind;
}

impl embedded_hal::i2c::I2c<SevenBitAddress> for FakeI2c<'_> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.run(address, operations)
    }
}

impl embedded_hal_async::i2c::I2c<SevenBitAddress> for FakeI2c<'_> {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.run(address, operations)
    }
}
