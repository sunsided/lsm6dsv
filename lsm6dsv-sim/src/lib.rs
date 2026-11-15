//! # STMicroelectronics LSM6DSV / LSM6DSV16X behavioral model
//!
//! A register-level model of the LSM6DSV IMU, used as a fake I²C bus ([`FakeI2c`]) or SPI
//! device ([`FakeSpi`]) in host tests of the `lsm6dsv` driver, with a delay ([`FakeDelay`])
//! that advances the model's time.
//!
//! The model implements:
//!
//! - the I²C and SPI protocols (register address, then auto-incrementing data while
//!   `CTRL3.IF_INC` is set),
//! - the main, embedded functions and sensor hub banks, selected via `FUNC_CFG_ACCESS`,
//! - the indirectly addressed embedded advanced feature pages (`PAGE_SEL`, `PAGE_ADDRESS`,
//!   `PAGE_VALUE`, `PAGE_RW`),
//! - accelerometer, gyroscope, temperature and Qvar outputs with their data-ready flags,
//!   set by the test via [`Lsm6dsvModel::set_accel`] and friends,
//! - the timestamp counter while `FUNCTIONS_ENABLE.TIMESTAMP_EN` is set,
//! - a FIFO of tagged words filled via [`Lsm6dsvModel::push_fifo`], with `DIFF_FIFO`,
//!   the watermark flag and popping on reads of `FIFO_DATA_OUT_TAG`,
//! - the step counter and its reset (`EMB_FUNC_SRC.PEDO_RST_STEP`),
//! - a new sample (data-ready flag) on every [`Lsm6dsvModel::tick_ns`] while the
//!   accelerometer or gyroscope ODR is not power-down,
//! - self-test, which adds [`MODEL_SELF_TEST_LSB`] to every axis (subtracts it for
//!   negative self-test),
//! - software reset (`CTRL3.SW_RESET`) and software power-on reset (`FUNC_CFG_ACCESS.SW_POR`).
//!
//! ## Not modelled
//!
//! Sensor physics: output data rates, filters, full-scale changes, the embedded functions
//! and interrupts. Their configuration bits can be written and read back, but do not
//! change the output. Read-only registers ignore writes.

#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]

mod fake_delay;
mod fake_i2c;
mod fake_spi;

pub use fake_delay::FakeDelay;
pub use fake_i2c::FakeI2c;
pub use fake_spi::FakeSpi;

use lsm6dsv_registers::{
    DEFAULT_DEVICE_ADDRESS, EmbeddedRegisterAddress as Emb, LSM6DSV_WHO_AM_I,
    RegisterAddress as Reg,
};

/// The FIFO capacity of the model, in words (tag plus six data bytes).
pub const FIFO_CAPACITY: usize = 512;

/// The output change the model applies to every axis during self-test, in LSB.
///
/// 4096 LSB are 500 mg at ±4 g and 287 dps at ±2000 dps, within the datasheet limits.
pub const MODEL_SELF_TEST_LSB: i16 = 4096;

/// The SPI read bit.
const SPI_READ: u8 = 0x80;

/// The phase of an SPI transfer.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum SpiState {
    /// The next byte holds the read bit and the register address.
    Command,
    /// The transfer reads registers.
    Read,
    /// The transfer writes registers.
    Write,
}

/// The selected register bank.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum Bank {
    Main,
    Embedded,
    SensorHub,
}

/// A register-level model of the LSM6DSV.
#[derive(Debug, Clone)]
pub struct Lsm6dsvModel {
    address: u8,
    main: [u8; 128],
    embedded: [u8; 128],
    sensor_hub: [u8; 128],
    pages: [[u8; 256]; 3],
    fifo: [[u8; 7]; FIFO_CAPACITY],
    fifo_head: usize,
    fifo_len: usize,
    pointer: u8,
    /// The next I²C write byte is the register address.
    expect_address: bool,
    spi: SpiState,
    /// Microseconds not yet counted by the timestamp.
    timestamp_remainder_ns: u64,
    steps: u16,
    accel: [i16; 3],
    gyro: [i16; 3],
}

impl Default for Lsm6dsvModel {
    fn default() -> Self {
        Self::new()
    }
}

impl Lsm6dsvModel {
    /// Creates a model in its power-on state at [`DEFAULT_DEVICE_ADDRESS`].
    pub fn new() -> Self {
        let mut model = Self {
            address: DEFAULT_DEVICE_ADDRESS,
            main: [0; 128],
            embedded: [0; 128],
            sensor_hub: [0; 128],
            pages: [[0; 256]; 3],
            fifo: [[0; 7]; FIFO_CAPACITY],
            fifo_head: 0,
            fifo_len: 0,
            pointer: 0,
            expect_address: true,
            spi: SpiState::Command,
            timestamp_remainder_ns: 0,
            steps: 0,
            accel: [0; 3],
            gyro: [0; 3],
        };
        model.power_on_reset();
        model
    }

    /// Creates a model at the specified 7-bit I²C address.
    pub fn with_address(address: u8) -> Self {
        Self {
            address,
            ..Self::new()
        }
    }

    /// The 7-bit I²C address.
    pub fn address(&self) -> u8 {
        self.address
    }

    fn power_on_reset(&mut self) {
        self.main = [0; 128];
        self.embedded = [0; 128];
        self.sensor_hub = [0; 128];
        self.pages = [[0; 256]; 3];
        self.fifo_len = 0;
        self.fifo_head = 0;
        self.steps = 0;
        self.main[Reg::PIN_CTRL as usize] = 0x23;
        self.embedded[Emb::PAGE_SEL as usize] = 0x01;
        self.embedded[Emb::SFLP_ODR as usize] = 0x5B;
        self.embedded[Emb::FSM_ODR as usize] = 0x4B;
        self.software_reset();
    }

    /// Resets the control registers, as `CTRL3.SW_RESET` does.
    fn software_reset(&mut self) {
        for addr in 0x02..=0x19 {
            if addr != Reg::PIN_CTRL as usize && addr != Reg::IF_CFG as usize {
                self.main[addr] = 0;
            }
        }

        for addr in 0x50..=0x75 {
            self.main[addr] = 0;
        }

        self.main[Reg::WHO_AM_I as usize] = LSM6DSV_WHO_AM_I;
        self.main[Reg::CTRL3 as usize] = 0x44;
        self.main[Reg::DEN as usize] = 0x0E;
        self.main[Reg::INACTIVITY_DUR as usize] = 0x04;
        self.fifo_len = 0;
        self.update_fifo_status();
    }

    // --- test-facing API ---------------------------------------------------------------------

    /// Sets the raw accelerometer output and the `XLDA` flag.
    pub fn set_accel(&mut self, raw: [i16; 3]) {
        self.accel = raw;
        self.refresh_outputs();
        self.main[Reg::STATUS_REG as usize] |= 0b001;
    }

    /// Sets the raw gyroscope output and the `GDA` flag.
    pub fn set_gyro(&mut self, raw: [i16; 3]) {
        self.gyro = raw;
        self.refresh_outputs();
        self.main[Reg::STATUS_REG as usize] |= 0b010;
    }

    /// Writes the accelerometer and gyroscope outputs, including the self-test change.
    fn refresh_outputs(&mut self) {
        let ctrl10 = self.main[Reg::CTRL10 as usize];
        let shift = |raw: [i16; 3], mode: u8| {
            let change = match mode {
                0b01 => MODEL_SELF_TEST_LSB,
                0b10 => -MODEL_SELF_TEST_LSB,
                _ => 0,
            };
            raw.map(|v| v.saturating_add(change))
        };
        let accel = shift(self.accel, ctrl10 & 0b11);
        let gyro = shift(self.gyro, (ctrl10 >> 2) & 0b11);
        self.set_words(Reg::OUTX_L_A as usize, &accel);
        self.set_words(Reg::OUTX_L_G as usize, &gyro);
    }

    /// Sets the raw temperature output and the `TDA` flag.
    pub fn set_temperature(&mut self, raw: i16) {
        self.set_words(Reg::OUT_TEMP_L as usize, &[raw]);
        self.main[Reg::STATUS_REG as usize] |= 0b100;
    }

    /// Sets the raw Qvar output (LSM6DSV16X) and the `AH_QVARDA` flag.
    pub fn set_qvar(&mut self, raw: i16) {
        self.set_words(0x3A, &[raw]);
        self.main[Reg::STATUS_REG as usize] |= 0b1000;
    }

    /// Sets the step counter.
    pub fn set_steps(&mut self, steps: u16) {
        self.steps = steps;
        let [l, h] = steps.to_le_bytes();
        self.embedded[Emb::STEP_COUNTER_L as usize] = l;
        self.embedded[Emb::STEP_COUNTER_H as usize] = h;
    }

    /// Sets the raw contents of a sensor hub output register (`SENSOR_HUB_1` is index 0).
    pub fn set_sensor_hub_data(&mut self, index: usize, data: &[u8]) {
        let start = 0x02 + index;
        self.sensor_hub[start..start + data.len()].copy_from_slice(data);
    }

    /// Appends a word to the FIFO. Returns `false` if the FIFO is full.
    pub fn push_fifo(&mut self, tag: u8, count: u8, data: [u8; 6]) -> bool {
        if self.fifo_len == FIFO_CAPACITY {
            return false;
        }

        let index = (self.fifo_head + self.fifo_len) % FIFO_CAPACITY;
        self.fifo[index][0] = (tag & 0x1F) << 3 | (count & 0b11) << 1;
        self.fifo[index][1..].copy_from_slice(&data);
        self.fifo_len += 1;
        self.update_fifo_status();
        true
    }

    /// The number of words in the FIFO.
    pub fn fifo_len(&self) -> usize {
        self.fifo_len
    }

    /// Reads a main bank register directly, without bus traffic.
    pub fn main_register(&self, addr: u8) -> u8 {
        self.main[addr as usize & 0x7F]
    }

    /// Reads an embedded functions bank register directly, without bus traffic.
    pub fn embedded_register(&self, addr: u8) -> u8 {
        self.embedded[addr as usize & 0x7F]
    }

    /// Reads a sensor hub bank register directly, without bus traffic.
    pub fn sensor_hub_register(&self, addr: u8) -> u8 {
        self.sensor_hub[addr as usize & 0x7F]
    }

    /// Reads an advanced features page register directly, without bus traffic.
    pub fn page_register(&self, page: u8, addr: u8) -> u8 {
        self.pages[page as usize % 3][addr as usize]
    }

    /// Advances the model's time.
    pub fn tick_ns(&mut self, ns: u64) {
        // A new sample arrives while the ODR (bits 3:0 of CTRL1/CTRL2) is not power-down.
        if self.main[Reg::CTRL1 as usize] & 0x0F != 0 {
            self.main[Reg::STATUS_REG as usize] |= 0b001;
        }

        if self.main[Reg::CTRL2 as usize] & 0x0F != 0 {
            self.main[Reg::STATUS_REG as usize] |= 0b010;
        }

        // FUNCTIONS_ENABLE.TIMESTAMP_EN is bit 6.
        if self.main[Reg::FUNCTIONS_ENABLE as usize] & 0x40 == 0 {
            return;
        }

        // 1 LSB = 21.75 µs.
        let total = self.timestamp_remainder_ns + ns;
        let lsb = total / 21_750;
        self.timestamp_remainder_ns = total % 21_750;
        let start = Reg::TIMESTAMP0 as usize;
        let mut bytes = [0; 4];
        bytes.copy_from_slice(&self.main[start..start + 4]);
        let value = u32::from_le_bytes(bytes).wrapping_add(lsb as u32);
        self.main[start..start + 4].copy_from_slice(&value.to_le_bytes());
    }

    // --- bus-facing API ----------------------------------------------------------------------

    /// Starts an I²C transfer. Returns whether the device acknowledges.
    pub fn i2c_start(&mut self, read: bool) -> bool {
        if !read {
            self.expect_address = true;
        }

        true
    }

    /// Receives a byte in an I²C write transfer. Returns whether the device acknowledges.
    pub fn i2c_write(&mut self, byte: u8) -> bool {
        if self.expect_address {
            self.pointer = byte & 0x7F;
            self.expect_address = false;
        } else {
            self.write(byte);
        }

        true
    }

    /// Sends a byte in an I²C read transfer.
    pub fn i2c_read(&mut self) -> u8 {
        self.read()
    }

    /// Starts an SPI transfer (chip select asserted).
    pub fn spi_select(&mut self) {
        self.spi = SpiState::Command;
    }

    /// Exchanges a byte on SPI.
    pub fn spi_transfer(&mut self, mosi: u8) -> u8 {
        match self.spi {
            SpiState::Command => {
                self.pointer = mosi & 0x7F;
                self.spi = if mosi & SPI_READ != 0 {
                    SpiState::Read
                } else {
                    SpiState::Write
                };
                0
            }

            SpiState::Read => self.read(),
            SpiState::Write => {
                self.write(mosi);
                0
            }
        }
    }

    // --- register file -----------------------------------------------------------------------

    fn set_words(&mut self, start: usize, values: &[i16]) {
        for (i, v) in values.iter().enumerate() {
            self.main[start + 2 * i..start + 2 * i + 2].copy_from_slice(&v.to_le_bytes());
        }
    }

    fn bank(&self) -> Bank {
        let access = self.main[Reg::FUNC_CFG_ACCESS as usize];
        if access & 0x80 != 0 {
            Bank::Embedded
        } else if access & 0x40 != 0 {
            Bank::SensorHub
        } else {
            Bank::Main
        }
    }

    fn advance(&mut self) {
        if self.main[Reg::CTRL3 as usize] & 0b100 != 0 {
            self.pointer = (self.pointer + 1) & 0x7F;
        }
    }

    fn read(&mut self) -> u8 {
        let addr = self.pointer;
        let value = if addr == Reg::FUNC_CFG_ACCESS as u8 {
            self.main[addr as usize]
        } else {
            match self.bank() {
                Bank::Main => self.read_main(addr),
                Bank::Embedded => self.read_embedded(addr),
                Bank::SensorHub => self.sensor_hub[addr as usize],
            }
        };
        self.advance();
        value
    }

    fn read_main(&mut self, addr: u8) -> u8 {
        if addr == Reg::FIFO_DATA_OUT_TAG as u8 {
            self.pop_fifo();
        }

        let value = self.main[addr as usize];
        // Reading the high byte of an output clears its data-ready flag.
        let status = &mut self.main[Reg::STATUS_REG as usize];
        match addr {
            0x21 => *status &= !0b100,
            0x27 => *status &= !0b010,
            0x2D => *status &= !0b001,
            0x3B => *status &= !0b1000,
            _ => {}
        }

        value
    }

    fn read_embedded(&mut self, addr: u8) -> u8 {
        if addr == Emb::PAGE_VALUE as u8 && self.embedded[Emb::PAGE_RW as usize] & 0x20 != 0 {
            let (page, offset) = self.page_cursor();
            let value = self.pages[page][offset as usize];
            self.advance_page_cursor();
            return value;
        }

        self.embedded[addr as usize]
    }

    fn write(&mut self, value: u8) {
        let addr = self.pointer;
        if addr == Reg::FUNC_CFG_ACCESS as u8 {
            self.write_func_cfg_access(value);
        } else {
            match self.bank() {
                Bank::Main => self.write_main(addr, value),
                Bank::Embedded => self.write_embedded(addr, value),
                Bank::SensorHub => {
                    // SENSOR_HUB_1..18 and STATUS_MASTER are read-only.
                    if !(0x02..=0x13).contains(&addr) && addr != 0x22 {
                        self.sensor_hub[addr as usize] = value;
                    }
                }
            }
        }

        self.advance();
    }

    fn write_func_cfg_access(&mut self, value: u8) {
        // SW_POR is bit 2.
        if value & 0b100 != 0 {
            self.power_on_reset();
            return;
        }

        self.main[Reg::FUNC_CFG_ACCESS as usize] = value;
    }

    fn write_main(&mut self, addr: u8, value: u8) {
        let read_only = addr == Reg::WHO_AM_I as u8
            || (0x1A..=0x4F).contains(&addr)
            || (0x78..=0x7E).contains(&addr);
        if read_only {
            return;
        }

        if addr == Reg::CTRL3 as u8 && value & 0b1 != 0 {
            self.software_reset();
            return;
        }

        // BOOT (bit 7) completes instantly.
        let value = if addr == Reg::CTRL3 as u8 {
            value & 0x7F
        } else {
            value
        };
        self.main[addr as usize] = value;
        if addr == Reg::FIFO_CTRL1 as u8 {
            self.update_fifo_status();
        }

        if addr == Reg::CTRL10 as u8 {
            self.refresh_outputs();
        }
    }

    fn write_embedded(&mut self, addr: u8, value: u8) {
        if addr == Emb::PAGE_VALUE as u8 && self.embedded[Emb::PAGE_RW as usize] & 0x40 != 0 {
            let (page, offset) = self.page_cursor();
            self.pages[page][offset as usize] = value;
            self.advance_page_cursor();
            return;
        }

        if addr == Emb::EMB_FUNC_SRC as u8 && value & 0x80 != 0 {
            // PEDO_RST_STEP (bit 7) resets the step counter.
            self.set_steps(0);
            self.embedded[addr as usize] = value & 0x7F;
            return;
        }

        let read_only = [
            Emb::EMB_FUNC_EXEC_STATUS as u8,
            Emb::EMB_FUNC_STATUS as u8,
            Emb::FSM_STATUS as u8,
            Emb::STEP_COUNTER_L as u8,
            Emb::STEP_COUNTER_H as u8,
        ];
        if !read_only.contains(&addr) {
            self.embedded[addr as usize] = value;
        }
    }

    fn page_cursor(&self) -> (usize, u8) {
        let page = (self.embedded[Emb::PAGE_SEL as usize] >> 4) as usize % 3;
        (page, self.embedded[Emb::PAGE_ADDRESS as usize])
    }

    fn advance_page_cursor(&mut self) {
        // The address wraps within the page; the host selects the next page itself.
        let offset = self.embedded[Emb::PAGE_ADDRESS as usize].wrapping_add(1);
        self.embedded[Emb::PAGE_ADDRESS as usize] = offset;
    }

    fn pop_fifo(&mut self) {
        let start = Reg::FIFO_DATA_OUT_TAG as usize;
        if self.fifo_len == 0 {
            self.main[start..start + 7].fill(0);
            return;
        }

        let word = self.fifo[self.fifo_head];
        self.main[start..start + 7].copy_from_slice(&word);
        self.fifo_head = (self.fifo_head + 1) % FIFO_CAPACITY;
        self.fifo_len -= 1;
        self.update_fifo_status();
    }

    fn update_fifo_status(&mut self) {
        let level = self.fifo_len as u16;
        let watermark = self.main[Reg::FIFO_CTRL1 as usize] as u16;
        let [low, high] = level.to_le_bytes();
        self.main[Reg::FIFO_STATUS1 as usize] = low;
        let mut status2 = high & 0b1;
        if watermark != 0 && level >= watermark {
            status2 |= 0x80;
        }

        if self.fifo_len == FIFO_CAPACITY {
            status2 |= 0x40;
        }

        self.main[Reg::FIFO_STATUS2 as usize] = status2;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::i2c::I2c;
    use embedded_hal::spi::{Operation, SpiDevice};

    #[test]
    fn i2c_who_am_i() {
        let model = core::cell::RefCell::new(Lsm6dsvModel::new());
        let mut bus = FakeI2c::new(&model);
        let mut id = [0];
        bus.write_read(0x6A, &[0x0F], &mut id).unwrap();
        assert_eq!(id, [0x70]);
        assert!(bus.write_read(0x6B, &[0x0F], &mut id).is_err());
    }

    #[test]
    fn spi_burst() {
        let model = core::cell::RefCell::new(Lsm6dsvModel::new());
        model.borrow_mut().set_accel([1, -2, 3]);
        let mut spi = FakeSpi::new(&model);
        let mut buf = [0; 6];
        spi.transaction(&mut [Operation::Write(&[0x80 | 0x28]), Operation::Read(&mut buf)])
            .unwrap();
        assert_eq!(buf, [1, 0, 0xFE, 0xFF, 3, 0]);
    }

    #[test]
    fn banks_and_pages() {
        let model = core::cell::RefCell::new(Lsm6dsvModel::new());
        let mut bus = FakeI2c::new(&model);
        // Embedded bank: write page 1, address 0x83 via PAGE_RW/PAGE_SEL/PAGE_ADDRESS/PAGE_VALUE.
        bus.write(0x6A, &[0x01, 0x80]).unwrap();
        bus.write(0x6A, &[0x17, 0x40]).unwrap();
        bus.write(0x6A, &[0x02, 0x11]).unwrap();
        bus.write(0x6A, &[0x08, 0x83]).unwrap();
        bus.write(0x6A, &[0x09, 0x5A]).unwrap();
        bus.write(0x6A, &[0x01, 0x00]).unwrap();
        assert_eq!(model.borrow().page_register(1, 0x83), 0x5A);
        // Main bank again.
        let mut id = [0];
        bus.write_read(0x6A, &[0x0F], &mut id).unwrap();
        assert_eq!(id, [0x70]);
    }

    #[test]
    fn fifo_pops_on_tag_read() {
        let model = core::cell::RefCell::new(Lsm6dsvModel::new());
        model.borrow_mut().push_fifo(0x02, 1, [1, 2, 3, 4, 5, 6]);
        let mut bus = FakeI2c::new(&model);
        let mut status = [0; 2];
        bus.write_read(0x6A, &[0x1B], &mut status).unwrap();
        assert_eq!(status, [1, 0]);
        let mut word = [0; 7];
        bus.write_read(0x6A, &[0x78], &mut word).unwrap();
        assert_eq!(word, [0x02 << 3 | 1 << 1, 1, 2, 3, 4, 5, 6]);
        assert_eq!(model.borrow().fifo_len(), 0);
    }

    #[test]
    fn software_reset_restores_defaults() {
        let model = core::cell::RefCell::new(Lsm6dsvModel::new());
        let mut bus = FakeI2c::new(&model);
        bus.write(0x6A, &[0x10, 0x56]).unwrap();
        bus.write(0x6A, &[0x12, 0x01]).unwrap();
        assert_eq!(model.borrow().main_register(0x10), 0);
        assert_eq!(model.borrow().main_register(0x12), 0x44);
    }

    #[test]
    fn async_bus() {
        let model = core::cell::RefCell::new(Lsm6dsvModel::new());
        let mut bus = FakeI2c::new(&model);
        let mut id = [0];
        embassy_futures::block_on(embedded_hal_async::i2c::I2c::write_read(
            &mut bus,
            0x6A,
            &[0x0F],
            &mut id,
        ))
        .unwrap();
        assert_eq!(id, [0x70]);
    }
}
