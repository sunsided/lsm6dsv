//! An `embedded-hal` delay that advances a [`Lsm6dsvModel`].

use crate::Lsm6dsvModel;
use core::cell::RefCell;

/// A delay provider that advances the time of a [`Lsm6dsvModel`] instead of waiting.
#[derive(Debug, Clone, Copy)]
pub struct FakeDelay<'a> {
    model: &'a RefCell<Lsm6dsvModel>,
}

impl<'a> FakeDelay<'a> {
    /// Creates a delay provider for the specified model.
    pub fn new(model: &'a RefCell<Lsm6dsvModel>) -> Self {
        Self { model }
    }
}

impl embedded_hal::delay::DelayNs for FakeDelay<'_> {
    fn delay_ns(&mut self, ns: u32) {
        self.model.borrow_mut().tick_ns(u64::from(ns));
    }
}

impl embedded_hal_async::delay::DelayNs for FakeDelay<'_> {
    async fn delay_ns(&mut self, ns: u32) {
        self.model.borrow_mut().tick_ns(u64::from(ns));
    }
}
