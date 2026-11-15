//! The driver body, shared by the blocking and the async driver.
//!
//! [`define_driver!`] expands once with plain functions on [`Interface`](crate::Interface)
//! and once with `async` functions on [`AsyncInterface`](crate::AsyncInterface), inserting
//! `.await` after every bus access. All decoding lives in plain functions outside the macro.

use crate::{FifoStatus, FifoWord};
use lsm6dsv_registers::{
    AccelFullScale, Bank, Ctrl6, Ctrl8, FuncCfgAccess, GyroFullScale, PageSel, RegisterAddress,
};

/// The number of 1 ms polls [`reset`](crate::Lsm6dsv::reset) and
/// [`reboot`](crate::Lsm6dsv::reboot) wait for the device.
pub const RESET_POLLS: u32 = 50;

/// The address of `FUNC_CFG_ACCESS`, which is reachable from every bank.
pub(crate) const FUNC_CFG_ACCESS: u8 = RegisterAddress::FUNC_CFG_ACCESS.addr();

/// An error of an operation that waits for the device.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// A bus error.
    Bus(E),
    /// The device did not finish in time.
    Timeout,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Self::Bus(error)
    }
}

/// The full scales the driver uses to convert samples, if known.
#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
pub(crate) struct Scales {
    pub accel: Option<AccelFullScale>,
    pub gyro: Option<GyroFullScale>,
}

impl Scales {
    /// Updates the tracked full scales after writing `data` to the main bank at `start`.
    pub fn track(&mut self, start: u8, data: &[u8]) {
        for (i, &value) in data.iter().enumerate() {
            let addr = start.wrapping_add(i as u8);
            if addr == RegisterAddress::CTRL6.addr() {
                self.gyro = Some(Ctrl6::from(value).fs_g());
            } else if addr == RegisterAddress::CTRL8.addr() {
                self.accel = Some(Ctrl8::from(value).fs_xl());
            } else if addr == RegisterAddress::CTRL3.addr() && value & 0b1 != 0 {
                // Software reset restores the default full scales.
                *self = Self {
                    accel: Some(AccelFullScale::G2),
                    gyro: Some(GyroFullScale::Dps125),
                };
            }
        }
    }
}

/// Returns `(FUNC_CFG_ACCESS for the main bank, FUNC_CFG_ACCESS selecting bank)` for a
/// current `FUNC_CFG_ACCESS` value. Keeps the other bits.
pub(crate) fn bank_selection(current: u8, bank: Bank) -> (u8, u8) {
    let main = FuncCfgAccess::from(current)
        .with_emb_func_reg_access(false)
        .with_shub_reg_access(false)
        .with_sw_por(false);
    let (embedded, sensor_hub) = bank.selection();
    let selected = main
        .with_emb_func_reg_access(embedded)
        .with_shub_reg_access(sensor_hub);
    (main.into(), selected.into())
}

/// The `PAGE_SEL` value selecting an advanced features page.
pub(crate) fn page_select(page: u8) -> u8 {
    // Bit 0 must stay 1 (its default value).
    (PageSel::new().with_page_sel(page).into_bits()) | 0b1
}

/// The number of whole FIFO words to read, given the level and the buffer length.
pub(crate) fn fifo_words_to_read(status: FifoStatus, capacity: usize) -> usize {
    usize::from(status.level).min(capacity)
}

pub(crate) fn fifo_word(bytes: &[u8; 7]) -> FifoWord {
    FifoWord::from_bytes(bytes)
}

/// Defines the driver struct and its methods.
///
/// `$iface` is the interface trait, `$delay` the delay trait, `$($async)?` is empty or
/// `async`, and `$($await)*` is empty or `.await`.
macro_rules! define_driver {
    ($iface:path, $delay:path, [$($async:tt)?], [$($await:tt)*]) => {
        use $crate::driver::{Error, Scales, FUNC_CFG_ACCESS, RESET_POLLS};
        use $crate::interface::{I2cInterface, SpiInterface};
        use $crate::{
            Acceleration, AngularRate, FifoStatus, FifoWord, Temperature, Timestamp,
        };
        use lsm6dsv_registers::*;

        /// LSM6DSV / LSM6DSV16X driver.
        ///
        /// The driver keeps the device in the main register bank between calls. Accesses to
        /// registers of other banks switch the bank and switch back, even if the access fails.
        #[derive(Debug)]
        pub struct Lsm6dsv<IFACE> {
            iface: IFACE,
            scales: Scales,
        }

        impl<I2C> Lsm6dsv<I2cInterface<I2C>> {
            /// Creates a driver for a device on I²C at [`DEFAULT_DEVICE_ADDRESS`].
            /// Performs no bus access.
            pub fn new(i2c: I2C) -> Self {
                Self::from_interface(I2cInterface::new(i2c))
            }

            /// Creates a driver for a device on I²C at the specified 7-bit address, such as
            /// [`ALTERNATIVE_DEVICE_ADDRESS`]. Performs no bus access.
            pub fn with_address(i2c: I2C, address: u8) -> Self {
                Self::from_interface(I2cInterface::with_address(i2c, address))
            }

            /// Consumes the driver and returns the I²C bus.
            pub fn release(self) -> I2C {
                self.iface.release()
            }
        }

        impl<SPI> Lsm6dsv<SpiInterface<SPI>> {
            /// Creates a driver for a device on SPI. Performs no bus access.
            pub fn new_spi(spi: SPI) -> Self {
                Self::from_interface(SpiInterface::new(spi))
            }

            /// Consumes the driver and returns the SPI device.
            pub fn release(self) -> SPI {
                self.iface.release()
            }
        }

        impl<IFACE> Lsm6dsv<IFACE> {
            /// Creates a driver on the specified interface. Performs no bus access.
            pub fn from_interface(iface: IFACE) -> Self {
                Self {
                    iface,
                    scales: Scales::default(),
                }
            }

            /// Consumes the driver and returns the interface.
            pub fn into_interface(self) -> IFACE {
                self.iface
            }
        }

        impl<IFACE> Lsm6dsv<IFACE>
        where
            IFACE: $iface,
        {
            // --- identification and reset ----------------------------------------------------

            /// Reads the `WHO_AM_I` register.
            pub $($async)? fn who_am_i(&mut self) -> Result<u8, IFACE::Error> {
                Ok(self.read_register::<WhoAmI>() $($await)* ?.id())
            }

            /// Checks whether `WHO_AM_I` matches [`LSM6DSV_WHO_AM_I`].
            pub $($async)? fn identify(&mut self) -> Result<bool, IFACE::Error> {
                Ok(self.who_am_i() $($await)* ? == LSM6DSV_WHO_AM_I)
            }

            /// Resets the control registers to their defaults and waits until the device is done.
            ///
            /// Powers down the accelerometer and gyroscope first, as the datasheet requires.
            pub $($async)? fn reset<D: $delay>(&mut self, delay: &mut D) -> Result<(), Error<IFACE::Error>> {
                self.modify_register(|r: Ctrl1| r.with_odr_xl(DataRate::Off)) $($await)* ?;
                self.modify_register(|r: Ctrl2| r.with_odr_g(DataRate::Off)) $($await)* ?;
                self.write_register(Ctrl3::new().with_sw_reset(true)) $($await)* ?;
                for _ in 0..RESET_POLLS {
                    delay.delay_ms(1) $($await)*;
                    if !self.read_register::<Ctrl3>() $($await)* ?.sw_reset() {
                        return Ok(());
                    }
                }
                Err(Error::Timeout)
            }

            /// Reloads the trimming parameters from non-volatile memory and waits until the
            /// device is done. Keeps the configuration.
            pub $($async)? fn reboot<D: $delay>(&mut self, delay: &mut D) -> Result<(), Error<IFACE::Error>> {
                self.modify_register(|r: Ctrl3| r.with_boot(true)) $($await)* ?;
                for _ in 0..RESET_POLLS {
                    delay.delay_ms(1) $($await)*;
                    if !self.read_register::<Ctrl3>() $($await)* ?.boot() {
                        return Ok(());
                    }
                }
                Err(Error::Timeout)
            }

            /// Performs a software power-on reset: resets all registers, including the
            /// embedded functions, and reloads the trimming parameters. Wait for the turn-on
            /// time (30 ms for the gyroscope) before using the device.
            pub $($async)? fn power_on_reset(&mut self) -> Result<(), IFACE::Error> {
                self.scales = Scales::default();
                self.iface.write(FUNC_CFG_ACCESS, &[FuncCfgAccess::new().with_sw_por(true).into()]) $($await)*
            }

            /// Enables block data update (the power-on default): output registers are not
            /// updated until both bytes of a value have been read.
            pub $($async)? fn set_block_data_update(&mut self, enabled: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: Ctrl3| r.with_bdu(enabled)) $($await)*
            }

            // --- accelerometer and gyroscope configuration -----------------------------------

            /// Sets the accelerometer output data rate and operating mode.
            pub $($async)? fn set_accel_mode(&mut self, rate: DataRate, mode: AccelMode) -> Result<(), IFACE::Error> {
                self.modify_register(|r: Ctrl1| r.with_odr_xl(rate).with_op_mode_xl(mode)) $($await)*
            }

            /// Sets the gyroscope output data rate and operating mode.
            pub $($async)? fn set_gyro_mode(&mut self, rate: DataRate, mode: GyroMode) -> Result<(), IFACE::Error> {
                self.modify_register(|r: Ctrl2| r.with_odr_g(rate).with_op_mode_g(mode)) $($await)*
            }

            /// Sets the accelerometer full scale.
            pub $($async)? fn set_accel_full_scale(&mut self, scale: AccelFullScale) -> Result<(), IFACE::Error> {
                self.modify_register(|r: Ctrl8| r.with_fs_xl(scale)) $($await)*
            }

            /// Sets the gyroscope full scale.
            pub $($async)? fn set_gyro_full_scale(&mut self, scale: GyroFullScale) -> Result<(), IFACE::Error> {
                self.modify_register(|r: Ctrl6| r.with_fs_g(scale)) $($await)*
            }

            /// Returns the accelerometer full scale. Reads `CTRL8` only if the driver does not know it.
            pub $($async)? fn accel_full_scale(&mut self) -> Result<AccelFullScale, IFACE::Error> {
                match self.scales.accel {
                    Some(scale) => Ok(scale),
                    None => {
                        let scale = self.read_register::<Ctrl8>() $($await)* ?.fs_xl();
                        self.scales.accel = Some(scale);
                        Ok(scale)
                    }
                }
            }

            /// Returns the gyroscope full scale. Reads `CTRL6` only if the driver does not know it.
            pub $($async)? fn gyro_full_scale(&mut self) -> Result<GyroFullScale, IFACE::Error> {
                match self.scales.gyro {
                    Some(scale) => Ok(scale),
                    None => {
                        let scale = self.read_register::<Ctrl6>() $($await)* ?.fs_g();
                        self.scales.gyro = Some(scale);
                        Ok(scale)
                    }
                }
            }

            /// Writes the accelerometer user offsets (X, Y, Z, two's complement).
            ///
            /// The weight is 2^-10 g/LSB, or 2^-6 g/LSB with `coarse`. The offsets apply to
            /// the output registers if `on_output` is set, otherwise only to the embedded
            /// functions.
            pub $($async)? fn set_accel_offset(&mut self, offset: [i8; 3], coarse: bool, on_output: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: Ctrl9| r.with_usr_off_w(coarse).with_usr_off_on_out(on_output)) $($await)* ?;
                self.write_bank(Bank::Main, RegisterAddress::X_OFS_USR.addr(), &offset.map(|v| v as u8)) $($await)*
            }

            // --- filters and self-test -------------------------------------------------------

            /// Configures the gyroscope LPF1: `None` disables it, `Some(code)` enables it with
            /// the `LPF1_G_BW` bandwidth code (0 to 7; the cutoff depends on the ODR, see the
            /// datasheet's LPF1 table).
            pub $($async)? fn set_gyro_low_pass(&mut self, bandwidth: Option<u8>) -> Result<(), IFACE::Error> {
                if let Some(code) = bandwidth {
                    self.modify_register(|r: Ctrl6| r.with_lpf1_g_bw(code)) $($await)* ?;
                }

                self.modify_register(|r: Ctrl7| r.with_lpf1_g_en(bandwidth.is_some())) $($await)*
            }

            /// Configures the accelerometer filter after LPF1.
            pub $($async)? fn set_accel_filter(&mut self, filter: $crate::AccelFilter) -> Result<(), IFACE::Error> {
                let (low_pass, high_pass, bandwidth) = filter.bits();
                self.modify_register(|r: Ctrl8| r.with_hp_lpf2_xl_bw(bandwidth)) $($await)* ?;
                self.modify_register(|r: Ctrl9| r.with_lpf2_xl_en(low_pass).with_hp_slope_xl_en(high_pass)) $($await)*
            }

            /// Sets the accelerometer and gyroscope self-test modes directly.
            ///
            /// See [`Self::self_test_accel`] and [`Self::self_test_gyro`] for the complete procedure.
            pub $($async)? fn set_self_test(&mut self, accel: SelfTestMode, gyro: SelfTestMode) -> Result<(), IFACE::Error> {
                self.modify_register(|r: Ctrl10| r.with_st_xl(accel).with_st_g(gyro)) $($await)*
            }

            /// Runs the accelerometer self-test and restores the configuration afterwards.
            ///
            /// Averages [`SELF_TEST_SAMPLES`](crate::SELF_TEST_SAMPLES) samples at 60 Hz and ±4 g
            /// without and with positive self-test. Takes about 0.4 s. The device must not move.
            pub $($async)? fn self_test_accel<D: $delay>(&mut self, delay: &mut D) -> Result<$crate::SelfTest, $crate::SelfTestError<IFACE::Error>> {
                let ctrl1 = self.read_register::<Ctrl1>() $($await)* ?;
                let ctrl8 = self.read_register::<Ctrl8>() $($await)* ?;
                let ctrl10 = self.read_register::<Ctrl10>() $($await)* ?;
                let result = self.run_self_test(true, delay) $($await)*;

                // Restore, even if the test failed.
                self.write_register(ctrl10) $($await)* ?;
                self.write_register(ctrl8) $($await)* ?;
                self.write_register(ctrl1) $($await)* ?;
                result
            }

            /// Runs the gyroscope self-test and restores the configuration afterwards.
            ///
            /// Averages [`SELF_TEST_SAMPLES`](crate::SELF_TEST_SAMPLES) samples at 240 Hz and
            /// ±2000 dps without and with positive self-test. Takes about 0.3 s. The device must
            /// not move.
            pub $($async)? fn self_test_gyro<D: $delay>(&mut self, delay: &mut D) -> Result<$crate::SelfTest, $crate::SelfTestError<IFACE::Error>> {
                let ctrl2 = self.read_register::<Ctrl2>() $($await)* ?;
                let ctrl6 = self.read_register::<Ctrl6>() $($await)* ?;
                let ctrl10 = self.read_register::<Ctrl10>() $($await)* ?;
                let result = self.run_self_test(false, delay) $($await)*;

                // Restore, even if the test failed.
                self.write_register(ctrl10) $($await)* ?;
                self.write_register(ctrl6) $($await)* ?;
                self.write_register(ctrl2) $($await)* ?;
                result
            }

            $($async)? fn run_self_test<D: $delay>(&mut self, accel: bool, delay: &mut D) -> Result<$crate::SelfTest, $crate::SelfTestError<IFACE::Error>> {
                use $crate::self_test::{ACCEL_SCALE, GYRO_SCALE, SETTLE_MS};

                if accel {
                    self.write_register(Ctrl8::new().with_fs_xl(ACCEL_SCALE)) $($await)* ?;
                    self.write_register(Ctrl1::new().with_odr_xl(DataRate::Hz60)) $($await)* ?;
                } else {
                    self.write_register(Ctrl6::new().with_fs_g(GYRO_SCALE)) $($await)* ?;
                    self.write_register(Ctrl2::new().with_odr_g(DataRate::Hz240)) $($await)* ?;
                }

                self.set_self_test_mode(accel, SelfTestMode::Normal) $($await)* ?;
                delay.delay_ms(SETTLE_MS) $($await)*;
                let without = self.self_test_sum(accel, delay) $($await)* ?;

                self.set_self_test_mode(accel, SelfTestMode::Positive) $($await)* ?;
                delay.delay_ms(SETTLE_MS) $($await)*;
                let with = self.self_test_sum(accel, delay) $($await)* ?;

                if accel {
                    Ok($crate::SelfTest::accel(without, with))
                } else {
                    Ok($crate::SelfTest::gyro(without, with))
                }
            }

            $($async)? fn set_self_test_mode(&mut self, accel: bool, mode: SelfTestMode) -> Result<(), IFACE::Error> {
                if accel {
                    self.modify_register(|r: Ctrl10| r.with_st_xl(mode)) $($await)*
                } else {
                    self.modify_register(|r: Ctrl10| r.with_st_g(mode)) $($await)*
                }
            }

            /// Discards one sample, then sums up [`SELF_TEST_SAMPLES`](crate::SELF_TEST_SAMPLES) raw samples.
            $($async)? fn self_test_sum<D: $delay>(&mut self, accel: bool, delay: &mut D) -> Result<[i32; 3], $crate::SelfTestError<IFACE::Error>> {
                let mut sum = [0; 3];

                for i in 0..=$crate::SELF_TEST_SAMPLES {
                    self.wait_for_sample(accel, delay) $($await)* ?;
                    let raw = if accel {
                        self.read_acceleration() $($await)* ?.raw
                    } else {
                        self.read_angular_rate() $($await)* ?.raw
                    };

                    if i > 0 {
                        $crate::self_test::accumulate(&mut sum, raw);
                    }
                }

                Ok(sum)
            }

            $($async)? fn wait_for_sample<D: $delay>(&mut self, accel: bool, delay: &mut D) -> Result<(), $crate::SelfTestError<IFACE::Error>> {
                for _ in 0..$crate::self_test::MAX_POLLS {
                    let status = self.status() $($await)* ?;
                    if (accel && status.xlda()) || (!accel && status.gda()) {
                        return Ok(());
                    }

                    delay.delay_ms($crate::self_test::POLL_MS) $($await)*;
                }

                Err($crate::SelfTestError::Timeout)
            }

            // --- output data -----------------------------------------------------------------

            /// Reads the status register (data-ready flags).
            pub $($async)? fn status(&mut self) -> Result<StatusReg, IFACE::Error> {
                self.read_register() $($await)*
            }

            /// Reads the accelerometer outputs in one burst.
            pub $($async)? fn read_acceleration(&mut self) -> Result<Acceleration, IFACE::Error> {
                let scale = self.accel_full_scale() $($await)* ?;
                let mut bytes = [0; 6];
                self.read_bank(Bank::Main, RegisterAddress::OUTX_L_A.addr(), &mut bytes) $($await)* ?;
                Ok(Acceleration::from_bytes(&bytes, scale))
            }

            /// Reads the gyroscope outputs in one burst.
            pub $($async)? fn read_angular_rate(&mut self) -> Result<AngularRate, IFACE::Error> {
                let scale = self.gyro_full_scale() $($await)* ?;
                let mut bytes = [0; 6];
                self.read_bank(Bank::Main, RegisterAddress::OUTX_L_G.addr(), &mut bytes) $($await)* ?;
                Ok(AngularRate::from_bytes(&bytes, scale))
            }

            /// Reads the temperature output.
            pub $($async)? fn read_temperature(&mut self) -> Result<Temperature, IFACE::Error> {
                let mut bytes = [0; 2];
                self.read_bank(Bank::Main, RegisterAddress::OUT_TEMP_L.addr(), &mut bytes) $($await)* ?;
                Ok(Temperature::from_bytes(bytes))
            }

            /// Enables the timestamp counter.
            pub $($async)? fn set_timestamp(&mut self, enabled: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: FunctionsEnable| r.with_timestamp_en(enabled)) $($await)*
            }

            /// Reads the timestamp counter.
            pub $($async)? fn read_timestamp(&mut self) -> Result<Timestamp, IFACE::Error> {
                let mut bytes = [0; 4];
                self.read_bank(Bank::Main, RegisterAddress::TIMESTAMP0.addr(), &mut bytes) $($await)* ?;
                Ok(Timestamp::from_bytes(bytes))
            }

            /// Enables the Qvar (electrostatic charge variation) channel (LSM6DSV16X).
            #[cfg(feature = "lsm6dsv16x")]
            #[cfg_attr(docsrs, doc(cfg(feature = "lsm6dsv16x")))]
            pub $($async)? fn set_qvar(&mut self, enabled: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: Ctrl7| r.with_ah_qvar_en(enabled)) $($await)*
            }

            /// Reads the Qvar output (LSM6DSV16X).
            #[cfg(feature = "lsm6dsv16x")]
            #[cfg_attr(docsrs, doc(cfg(feature = "lsm6dsv16x")))]
            pub $($async)? fn read_qvar(&mut self) -> Result<$crate::Qvar, IFACE::Error> {
                let mut bytes = [0; 2];
                self.read_bank(Bank::Main, RegisterAddress::AH_QVAR_OUT_L.addr(), &mut bytes) $($await)* ?;
                Ok($crate::Qvar { raw: i16::from_le_bytes(bytes) })
            }

            // --- FIFO ------------------------------------------------------------------------

            /// Sets the FIFO mode.
            pub $($async)? fn set_fifo_mode(&mut self, mode: FifoMode) -> Result<(), IFACE::Error> {
                self.modify_register(|r: FifoCtrl4| r.with_fifo_mode(mode)) $($await)*
            }

            /// Sets the FIFO watermark, in words.
            pub $($async)? fn set_fifo_watermark(&mut self, words: u8) -> Result<(), IFACE::Error> {
                self.write_register(FifoCtrl1::new().with_wtm(words)) $($await)*
            }

            /// Stops filling the FIFO at the watermark.
            pub $($async)? fn set_fifo_stop_on_watermark(&mut self, enabled: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: FifoCtrl2| r.with_stop_on_wtm(enabled)) $($await)*
            }

            /// Sets the accelerometer and gyroscope batch data rates ([`DataRate::Off`]: not batched).
            pub $($async)? fn set_fifo_batching(&mut self, accel: DataRate, gyro: DataRate) -> Result<(), IFACE::Error> {
                self.write_register(FifoCtrl3::new().with_bdr_xl(accel).with_bdr_gy(gyro)) $($await)*
            }

            /// Sets the temperature batch rate and the timestamp decimation.
            pub $($async)? fn set_fifo_extra_batching(&mut self, temperature: TemperatureBatchRate, timestamp: TimestampDecimation) -> Result<(), IFACE::Error> {
                self.modify_register(|r: FifoCtrl4| r.with_odr_t_batch(temperature).with_dec_ts_batch(timestamp)) $($await)*
            }

            /// Enables FIFO compression (in the embedded functions and at run time).
            ///
            /// Decode compressed words with a [`FifoDecoder`](crate::FifoDecoder).
            pub $($async)? fn set_fifo_compression(&mut self, enabled: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: EmbFuncEnB| r.with_fifo_compr_en(enabled)) $($await)* ?;
                self.modify_register(|r: FifoCtrl2| r.with_fifo_compr_rt_en(enabled)) $($await)*
            }

            /// Reads the FIFO fill level and flags.
            pub $($async)? fn fifo_status(&mut self) -> Result<FifoStatus, IFACE::Error> {
                let mut bytes = [0; 2];
                self.read_bank(Bank::Main, RegisterAddress::FIFO_STATUS1.addr(), &mut bytes) $($await)* ?;
                Ok(FifoStatus::from_bytes(bytes))
            }

            /// Reads one FIFO word. Returns a [`FifoTag::Empty`] word if the FIFO is empty.
            pub $($async)? fn read_fifo_word(&mut self) -> Result<FifoWord, IFACE::Error> {
                let mut bytes = [0; 7];
                self.read_bank(Bank::Main, RegisterAddress::FIFO_DATA_OUT_TAG.addr(), &mut bytes) $($await)* ?;
                Ok($crate::driver::fifo_word(&bytes))
            }

            /// Reads as many FIFO words as are stored, up to the buffer length.
            /// Returns the number of words read.
            pub $($async)? fn read_fifo(&mut self, words: &mut [FifoWord]) -> Result<usize, IFACE::Error> {
                let status = self.fifo_status() $($await)* ?;
                let count = $crate::driver::fifo_words_to_read(status, words.len());
                for word in &mut words[..count] {
                    *word = self.read_fifo_word() $($await)* ?;
                }
                Ok(count)
            }

            // --- interrupts ------------------------------------------------------------------

            /// Configures the interrupt pins: push-pull or open-drain, and active high or low.
            pub $($async)? fn set_interrupt_pins(&mut self, open_drain: bool, active_low: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: IfCfg| r.with_pp_od(open_drain).with_h_lactive(active_low)) $($await)*
            }

            /// Routes data-ready and FIFO signals to INT1.
            pub $($async)? fn set_int1_routing(&mut self, routing: Int1Ctrl) -> Result<(), IFACE::Error> {
                self.write_register(routing) $($await)*
            }

            /// Routes data-ready and FIFO signals to INT2.
            pub $($async)? fn set_int2_routing(&mut self, routing: Int2Ctrl) -> Result<(), IFACE::Error> {
                self.write_register(routing) $($await)*
            }

            /// Routes event interrupts (wake-up, tap, free-fall, 6D, embedded functions) to INT1.
            pub $($async)? fn set_int1_events(&mut self, routing: Md1Cfg) -> Result<(), IFACE::Error> {
                self.write_register(routing) $($await)*
            }

            /// Routes event interrupts (wake-up, tap, free-fall, 6D, embedded functions) to INT2.
            pub $($async)? fn set_int2_events(&mut self, routing: Md2Cfg) -> Result<(), IFACE::Error> {
                self.write_register(routing) $($await)*
            }

            /// Enables the event interrupts (`FUNCTIONS_ENABLE.INTERRUPTS_ENABLE`) and selects
            /// latched mode, where reading the source register clears an event.
            pub $($async)? fn set_event_interrupts(&mut self, enabled: bool, latched: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: TapCfg0| r.with_lir(latched)) $($await)* ?;
                self.modify_register(|r: FunctionsEnable| r.with_interrupts_enable(enabled)) $($await)*
            }

            /// Reads the combined interrupt source register.
            pub $($async)? fn interrupt_sources(&mut self) -> Result<AllIntSrc, IFACE::Error> {
                self.read_register() $($await)*
            }

            /// Reads the wake-up and free-fall source register.
            pub $($async)? fn wake_up_source(&mut self) -> Result<WakeUpSrc, IFACE::Error> {
                self.read_register() $($await)*
            }

            /// Reads the tap source register.
            pub $($async)? fn tap_source(&mut self) -> Result<TapSrc, IFACE::Error> {
                self.read_register() $($await)*
            }

            /// Reads the 6D orientation source register.
            pub $($async)? fn orientation_source(&mut self) -> Result<D6dSrc, IFACE::Error> {
                self.read_register() $($await)*
            }

            // --- motion events ---------------------------------------------------------------

            /// Configures wake-up detection: threshold (6 bits, in units of the full scale / 64
            /// or / 256, see `INACTIVITY_DUR.WU_INACT_THS_W`) and duration (2 bits, in ODR cycles).
            pub $($async)? fn set_wake_up(&mut self, threshold: u8, duration: u8) -> Result<(), IFACE::Error> {
                self.modify_register(|r: WakeUpThs| r.with_wk_ths(threshold)) $($await)* ?;
                self.modify_register(|r: WakeUpDur| r.with_wake_dur(duration)) $($await)*
            }

            /// Configures free-fall detection: threshold code (3 bits) and duration
            /// (6 bits, in ODR cycles).
            pub $($async)? fn set_free_fall(&mut self, threshold: u8, duration: u8) -> Result<(), IFACE::Error> {
                self.write_register(FreeFall::new().with_ff_ths(threshold).with_ff_dur(duration & 0x1F)) $($await)* ?;
                self.modify_register(|r: WakeUpDur| r.with_ff_dur(duration & 0x20 != 0)) $($await)*
            }

            /// Configures tap detection: enabled axes and their thresholds (5 bits each),
            /// and whether double taps are detected.
            pub $($async)? fn set_tap(&mut self, axes: [bool; 3], thresholds: [u8; 3], double_tap: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: TapCfg0| r.with_tap_x_en(axes[0]).with_tap_y_en(axes[1]).with_tap_z_en(axes[2])) $($await)* ?;
                self.modify_register(|r: TapCfg1| r.with_tap_ths_x(thresholds[0])) $($await)* ?;
                self.modify_register(|r: TapCfg2| r.with_tap_ths_y(thresholds[1])) $($await)* ?;
                self.modify_register(|r: TapThs6d| r.with_tap_ths_z(thresholds[2])) $($await)* ?;
                self.modify_register(|r: WakeUpThs| r.with_single_double_tap(double_tap)) $($await)*
            }

            /// Configures the tap timing windows: shock (2 bits), quiet (2 bits) and the
            /// maximum time between taps of a double tap (4 bits).
            pub $($async)? fn set_tap_timing(&mut self, shock: u8, quiet: u8, duration: u8) -> Result<(), IFACE::Error> {
                self.write_register(TapDur::new().with_shock(shock).with_quiet(quiet).with_dur(duration)) $($await)*
            }

            /// Configures 6D/4D orientation detection: threshold code (2 bits) and 4D mode.
            pub $($async)? fn set_orientation(&mut self, threshold: u8, four_d: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: TapThs6d| r.with_sixd_ths(threshold).with_d4d_en(four_d)) $($await)*
            }

            /// Configures activity/inactivity detection: mode (2 bits, `INACT_EN`),
            /// sleep duration (4 bits) and inactivity threshold (6 bits).
            pub $($async)? fn set_inactivity(&mut self, mode: u8, sleep_duration: u8, threshold: u8) -> Result<(), IFACE::Error> {
                self.modify_register(|r: WakeUpDur| r.with_sleep_dur(sleep_duration)) $($await)* ?;
                self.modify_register(|r: InactivityThs| r.with_inact_ths(threshold)) $($await)* ?;
                self.modify_register(|r: FunctionsEnable| r.with_inact_en(mode)) $($await)*
            }

            // --- embedded functions ----------------------------------------------------------

            /// Enables the pedometer (step detector and counter).
            pub $($async)? fn set_pedometer(&mut self, enabled: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: EmbFuncEnA| r.with_pedo_en(enabled)) $($await)*
            }

            /// Reads the step counter.
            pub $($async)? fn step_count(&mut self) -> Result<u16, IFACE::Error> {
                let mut bytes = [0; 2];
                self.read_bank(Bank::EmbeddedFunctions, EmbeddedRegisterAddress::STEP_COUNTER_L.addr(), &mut bytes) $($await)* ?;
                Ok(u16::from_le_bytes(bytes))
            }

            /// Resets the step counter.
            pub $($async)? fn reset_step_count(&mut self) -> Result<(), IFACE::Error> {
                self.modify_register(|r: EmbFuncSrc| r.with_pedo_rst_step(true)) $($await)*
            }

            /// Enables tilt detection.
            pub $($async)? fn set_tilt(&mut self, enabled: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: EmbFuncEnA| r.with_tilt_en(enabled)) $($await)*
            }

            /// Enables significant motion detection.
            pub $($async)? fn set_significant_motion(&mut self, enabled: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: EmbFuncEnA| r.with_sign_motion_en(enabled)) $($await)*
            }

            /// Enables the SFLP game rotation vector algorithm at the specified rate.
            pub $($async)? fn set_sflp(&mut self, enabled: bool, rate: SflpDataRate) -> Result<(), IFACE::Error> {
                self.modify_register(|r: SflpOdr| r.with_sflp_game_odr(rate)) $($await)* ?;
                self.modify_register(|r: EmbFuncEnA| r.with_sflp_game_en(enabled)) $($await)*
            }

            /// Selects which embedded function outputs are batched into the FIFO.
            pub $($async)? fn set_embedded_fifo_batching(&mut self, game_rotation: bool, gravity: bool, gyro_bias: bool, step_counter: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: EmbFuncFifoEnA| {
                    r.with_sflp_game_fifo_en(game_rotation)
                        .with_sflp_gravity_fifo_en(gravity)
                        .with_sflp_gbias_fifo_en(gyro_bias)
                        .with_step_counter_fifo_en(step_counter)
                }) $($await)*
            }

            /// Routes embedded function events (step, tilt, significant motion, FSM) to INT1.
            ///
            /// Also sets `MD1_CFG.INT1_EMB_FUNC` if any event is routed.
            pub $($async)? fn set_int1_embedded_events(&mut self, routing: EmbFuncInt1) -> Result<(), IFACE::Error> {
                let any = u8::from(routing) != 0;
                self.write_register(routing) $($await)* ?;
                self.modify_register(|r: Md1Cfg| r.with_int1_emb_func(any)) $($await)*
            }

            /// Routes embedded function events (step, tilt, significant motion, FSM) to INT2.
            ///
            /// Also sets `MD2_CFG.INT2_EMB_FUNC` if any event is routed.
            pub $($async)? fn set_int2_embedded_events(&mut self, routing: EmbFuncInt2) -> Result<(), IFACE::Error> {
                let any = u8::from(routing) != 0;
                self.write_register(routing) $($await)* ?;
                self.modify_register(|r: Md2Cfg| r.with_int2_emb_func(any)) $($await)*
            }

            /// Reads the embedded function event flags (main bank copy).
            pub $($async)? fn embedded_status(&mut self) -> Result<EmbFuncStatusMainpage, IFACE::Error> {
                self.read_register() $($await)*
            }

            /// Reads the finite state machine interrupt flags (main bank copy).
            pub $($async)? fn fsm_status(&mut self) -> Result<FsmStatusMainpage, IFACE::Error> {
                self.read_register() $($await)*
            }

            /// Reads the eight finite state machine output registers (`FSM_OUTS1` to `FSM_OUTS8`).
            pub $($async)? fn fsm_outputs(&mut self) -> Result<[u8; 8], IFACE::Error> {
                let mut bytes = [0; 8];
                self.read_bank(Bank::EmbeddedFunctions, EmbeddedRegisterAddress::FSM_OUTS1.addr(), &mut bytes) $($await)* ?;
                Ok(bytes)
            }

            /// Reads the machine learning core interrupt flags (main bank copy, LSM6DSV16X).
            #[cfg(feature = "lsm6dsv16x")]
            #[cfg_attr(docsrs, doc(cfg(feature = "lsm6dsv16x")))]
            pub $($async)? fn mlc_status(&mut self) -> Result<MlcStatusMainpage, IFACE::Error> {
                self.read_register() $($await)*
            }

            /// Reads the four machine learning core outputs (`MLC1_SRC` to `MLC4_SRC`, LSM6DSV16X).
            #[cfg(feature = "lsm6dsv16x")]
            #[cfg_attr(docsrs, doc(cfg(feature = "lsm6dsv16x")))]
            pub $($async)? fn mlc_outputs(&mut self) -> Result<[u8; 4], IFACE::Error> {
                let mut bytes = [0; 4];
                self.read_bank(Bank::EmbeddedFunctions, EmbeddedRegisterAddress::MLC1_SRC.addr(), &mut bytes) $($await)* ?;
                Ok(bytes)
            }

            /// Writes a configuration sequence of `(address, value)` pairs, as found in ST's
            /// `.ucf` files for FSM and MLC programs.
            ///
            /// The writes go to raw addresses in whatever bank is selected; the sequence selects
            /// banks through `FUNC_CFG_ACCESS` (address `0x01`) itself. The driver selects the
            /// main bank afterwards, so a sequence that ends in another bank is safe.
            pub $($async)? fn write_configuration(&mut self, sequence: &[(u8, u8)]) -> Result<(), IFACE::Error> {
                let result = self.write_sequence(sequence) $($await)*;
                let restore = self.select_main_bank() $($await)*;
                result.and(restore)
            }

            $($async)? fn write_sequence(&mut self, sequence: &[(u8, u8)]) -> Result<(), IFACE::Error> {
                for &(address, value) in sequence {
                    self.iface.write(address, &[value]) $($await)* ?;
                }
                Ok(())
            }

            $($async)? fn select_main_bank(&mut self) -> Result<(), IFACE::Error> {
                let mut current = [0];
                self.iface.read(FUNC_CFG_ACCESS, &mut current) $($await)* ?;
                let (main, _) = $crate::driver::bank_selection(current[0], Bank::Main);
                self.iface.write(FUNC_CFG_ACCESS, &[main]) $($await)*
            }

            // --- sensor hub ------------------------------------------------------------------

            /// Configures sensor hub slave `index` (0 to 3) to read `length` bytes (up to 7)
            /// from register `register` of the I²C device at `address`.
            ///
            /// The data appears in the `SENSOR_HUB_x` registers in slave order.
            ///
            /// # Panics
            ///
            /// If `index` is greater than 3.
            pub $($async)? fn set_sensor_hub_slave(&mut self, index: u8, address: u8, register: u8, length: u8) -> Result<(), IFACE::Error> {
                assert!(index < 4, "the sensor hub has four slaves");
                let base = SensorHubRegisterAddress::SLV0_ADD.addr() + 3 * index;
                // SLVx_ADD: address in bits 7:1, read bit 0. SLVx_CONFIG: NUMOP in bits 2:0.
                let mut config = [0];
                self.read_bank(Bank::SensorHub, base + 2, &mut config) $($await)* ?;
                let config = (config[0] & !0b111) | (length & 0b111);
                self.write_bank(Bank::SensorHub, base, &[address << 1 | 1, register, config]) $($await)*
            }

            /// Enables the sensor hub master with `slaves` slaves (1 to 4).
            pub $($async)? fn set_sensor_hub(&mut self, enabled: bool, slaves: u8) -> Result<(), IFACE::Error> {
                let count = slaves.clamp(1, 4) - 1;
                self.modify_register(|r: MasterConfig| r.with_aux_sens_on(count).with_master_on(enabled)) $($await)*
            }

            /// Reads the sensor hub output registers, starting at `SENSOR_HUB_1`.
            ///
            /// # Panics
            ///
            /// If `buffer` is longer than the 18 output registers.
            pub $($async)? fn read_sensor_hub(&mut self, buffer: &mut [u8]) -> Result<(), IFACE::Error> {
                assert!(buffer.len() <= 18, "the sensor hub has 18 output registers");
                self.read_bank(Bank::SensorHub, SensorHubRegisterAddress::SENSOR_HUB_1.addr(), buffer) $($await)*
            }

            /// Reads the sensor hub status (end of operation, NACKs).
            pub $($async)? fn sensor_hub_status(&mut self) -> Result<StatusMaster, IFACE::Error> {
                self.read_register() $($await)*
            }

            // --- register access -------------------------------------------------------------

            /// Reads a register of any bank.
            pub $($async)? fn read_register<R: Register>(&mut self) -> Result<R, IFACE::Error> {
                let mut buffer = [0];
                self.read_bank(R::BANK, R::ADDRESS, &mut buffer) $($await)* ?;
                Ok(R::from(buffer[0]))
            }

            /// Writes a register of any bank.
            pub $($async)? fn write_register<R: WritableRegister>(&mut self, register: R) -> Result<(), IFACE::Error> {
                self.write_bank(R::BANK, R::ADDRESS, &[register.into()]) $($await)*
            }

            /// Reads, modifies and writes a register of any bank.
            pub $($async)? fn modify_register<R, F>(&mut self, f: F) -> Result<(), IFACE::Error>
            where
                R: WritableRegister,
                F: FnOnce(R) -> R,
            {
                let register = self.read_register::<R>() $($await)* ?;
                self.write_register(f(register)) $($await)*
            }

            /// Reads consecutive registers of a bank, starting at `start`.
            pub $($async)? fn read_bank(&mut self, bank: Bank, start: u8, buffer: &mut [u8]) -> Result<(), IFACE::Error> {
                if bank == Bank::Main {
                    return self.iface.read(start, buffer) $($await)*;
                }
                let main = self.select_bank(bank) $($await)* ?;
                let result = self.iface.read(start, buffer) $($await)*;
                let restore = self.iface.write(FUNC_CFG_ACCESS, &[main]) $($await)*;
                result.and(restore)
            }

            /// Writes consecutive registers of a bank, starting at `start`.
            pub $($async)? fn write_bank(&mut self, bank: Bank, start: u8, data: &[u8]) -> Result<(), IFACE::Error> {
                if bank == Bank::Main {
                    self.iface.write(start, data) $($await)* ?;
                    self.scales.track(start, data);
                    return Ok(());
                }
                let main = self.select_bank(bank) $($await)* ?;
                let result = self.iface.write(start, data) $($await)*;
                let restore = self.iface.write(FUNC_CFG_ACCESS, &[main]) $($await)*;
                result.and(restore)
            }

            /// Selects a bank. Returns the `FUNC_CFG_ACCESS` value that selects the main bank again.
            $($async)? fn select_bank(&mut self, bank: Bank) -> Result<u8, IFACE::Error> {
                let mut current = [0];
                self.iface.read(FUNC_CFG_ACCESS, &mut current) $($await)* ?;
                let (main, selected) = $crate::driver::bank_selection(current[0], bank);
                self.iface.write(FUNC_CFG_ACCESS, &[selected]) $($await)* ?;
                Ok(main)
            }

            /// Reads an embedded advanced features register.
            pub $($async)? fn read_advanced<R: AdvancedRegister>(&mut self) -> Result<R, IFACE::Error> {
                let mut buffer = [0];
                self.read_pages(R::PAGE, R::ADDRESS, &mut buffer) $($await)* ?;
                Ok(R::from(buffer[0]))
            }

            /// Writes an embedded advanced features register.
            pub $($async)? fn write_advanced<R: WritableAdvancedRegister>(&mut self, register: R) -> Result<(), IFACE::Error> {
                self.write_pages(R::PAGE, R::ADDRESS, &[register.into()]) $($await)*
            }

            /// Reads consecutive embedded advanced features registers, starting at `address`
            /// in `page`. Crosses into the next page at the end of a page.
            pub $($async)? fn read_pages(&mut self, page: u8, address: u8, buffer: &mut [u8]) -> Result<(), IFACE::Error> {
                let main = self.select_bank(Bank::EmbeddedFunctions) $($await)* ?;
                let result = self.page_transfer(page, address, $crate::driver::PageData::Read(buffer)) $($await)*;
                let restore = self.iface.write(FUNC_CFG_ACCESS, &[main]) $($await)*;
                result.and(restore)
            }

            /// Writes consecutive embedded advanced features registers, starting at `address`
            /// in `page`. Crosses into the next page at the end of a page.
            pub $($async)? fn write_pages(&mut self, page: u8, address: u8, data: &[u8]) -> Result<(), IFACE::Error> {
                let main = self.select_bank(Bank::EmbeddedFunctions) $($await)* ?;
                let result = self.page_transfer(page, address, $crate::driver::PageData::Write(data)) $($await)*;
                let restore = self.iface.write(FUNC_CFG_ACCESS, &[main]) $($await)*;
                result.and(restore)
            }

            /// Transfers page data. Expects the embedded functions bank to be selected.
            $($async)? fn page_transfer(&mut self, mut page: u8, address: u8, mut data: $crate::driver::PageData<'_>) -> Result<(), IFACE::Error> {
                let rw_addr = EmbeddedRegisterAddress::PAGE_RW.addr();
                let sel_addr = EmbeddedRegisterAddress::PAGE_SEL.addr();
                let value_addr = EmbeddedRegisterAddress::PAGE_VALUE.addr();
                let mut rw = [0];
                self.iface.read(rw_addr, &mut rw) $($await)* ?;
                let rw = PageRw::from(rw[0]);
                let reading = matches!(data, $crate::driver::PageData::Read(_));
                self.iface.write(rw_addr, &[rw.with_page_read(reading).with_page_write(!reading).into()]) $($await)* ?;
                self.iface.write(sel_addr, &[$crate::driver::page_select(page)]) $($await)* ?;
                self.iface.write(EmbeddedRegisterAddress::PAGE_ADDRESS.addr(), &[address]) $($await)* ?;
                let mut offset = address;
                for i in 0..data.len() {
                    match &mut data {
                        $crate::driver::PageData::Read(buffer) => {
                            let mut byte = [0];
                            self.iface.read(value_addr, &mut byte) $($await)* ?;
                            buffer[i] = byte[0];
                        }
                        $crate::driver::PageData::Write(bytes) => {
                            self.iface.write(value_addr, &[bytes[i]]) $($await)* ?;
                        }
                    }
                    offset = offset.wrapping_add(1);
                    if offset == 0 {
                        page += 1;
                        self.iface.write(sel_addr, &[$crate::driver::page_select(page)]) $($await)* ?;
                    }
                }
                self.iface.write(sel_addr, &[$crate::driver::page_select(0)]) $($await)* ?;
                self.iface.write(rw_addr, &[rw.with_page_read(false).with_page_write(false).into()]) $($await)*
            }
        }
    };
}

pub(crate) use define_driver;

/// The data of an advanced features page transfer.
pub(crate) enum PageData<'a> {
    Read(&'a mut [u8]),
    Write(&'a [u8]),
}

impl PageData<'_> {
    pub fn len(&self) -> usize {
        match self {
            Self::Read(buffer) => buffer.len(),
            Self::Write(data) => data.len(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bank_selection_keeps_other_bits() {
        // OIS_CTRL_FROM_UI (bit 0) and FSM_WR_CTRL_EN (bit 3) survive; SW_POR (bit 2) is dropped.
        assert_eq!(
            bank_selection(0b0000_1101, Bank::EmbeddedFunctions),
            (0b1001, 0x89)
        );
        assert_eq!(bank_selection(0x80, Bank::SensorHub), (0x00, 0x40));
        assert_eq!(bank_selection(0x40, Bank::Main), (0x00, 0x00));
    }

    #[test]
    fn page_select_values() {
        assert_eq!(page_select(0), 0x01);
        assert_eq!(page_select(2), 0x21);
    }

    #[test]
    fn scale_tracking() {
        let mut scales = Scales::default();
        scales.track(RegisterAddress::CTRL6.addr(), &[0b0000_0100, 0, 0x03]);
        assert_eq!(scales.gyro, Some(GyroFullScale::Dps2000));
        assert_eq!(scales.accel, Some(AccelFullScale::G16));
        scales.track(RegisterAddress::CTRL3.addr(), &[0x01]);
        assert_eq!(scales.accel, Some(AccelFullScale::G2));
    }
}
