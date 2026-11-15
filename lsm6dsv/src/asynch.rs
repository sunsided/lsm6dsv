//! Async driver on `embedded-hal-async`.
//!
//! Mirrors the blocking [`Lsm6dsv`](crate::Lsm6dsv) on [`AsyncInterface`](crate::AsyncInterface):
//! both are generated from the same source.

crate::driver::define_driver!(
    crate::AsyncInterface,
    embedded_hal_async::delay::DelayNs,
    [async],
    [.await]
);
