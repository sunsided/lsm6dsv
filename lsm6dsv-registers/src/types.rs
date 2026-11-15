//! Value types used in register fields.
//!
//! Every type converts from any bit pattern. Codes the datasheet marks as reserved
//! are kept in a `Reserved` variant, so reading and writing back a register never changes it.

/// Declares a field enum with const `into_bits` / `from_bits` and a `Reserved` catch-all.
macro_rules! field_enum {
    (
        $(#[$meta:meta])*
        $name:ident, $mask:expr, {
            $( $(#[$vmeta:meta])* $variant:ident = $value:expr ),+ $(,)?
        }
    ) => {
        $(#[$meta])*
        #[derive(Debug, Copy, Clone, PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub enum $name {
            $( $(#[$vmeta])* $variant, )+
            /// A code the datasheet marks as reserved. Holds the raw field value.
            Reserved(u8),
        }

        impl $name {
            /// Converts the value into its register representation.
            pub const fn into_bits(self) -> u8 {
                match self {
                    $( Self::$variant => $value, )+
                    Self::Reserved(bits) => bits & $mask,
                }
            }

            /// Converts the lowest bits of a register value into this type.
            pub const fn from_bits(value: u8) -> Self {
                match value & $mask {
                    $( $value => Self::$variant, )+
                    bits => Self::Reserved(bits),
                }
            }
        }
    };
}

field_enum!(
    /// Output data rate of the accelerometer and gyroscope (`ODR_XL`, `ODR_G`), and the
    /// FIFO batch data rate (`BDR_XL`, `BDR_GY`).
    ///
    /// The available rates depend on the operating mode; see the datasheet ODR tables.
    DataRate, 0b1111, {
        /// Power-down, or not batched in FIFO. The default.
        Off = 0b0000,
        /// 1.875 Hz (accelerometer low-power modes and FIFO batching only).
        Hz1_875 = 0b0001,
        /// 7.5 Hz.
        Hz7_5 = 0b0010,
        /// 15 Hz.
        Hz15 = 0b0011,
        /// 30 Hz.
        Hz30 = 0b0100,
        /// 60 Hz.
        Hz60 = 0b0101,
        /// 120 Hz.
        Hz120 = 0b0110,
        /// 240 Hz.
        Hz240 = 0b0111,
        /// 480 Hz.
        Hz480 = 0b1000,
        /// 960 Hz.
        Hz960 = 0b1001,
        /// 1.92 kHz.
        Hz1920 = 0b1010,
        /// 3.84 kHz.
        Hz3840 = 0b1011,
        /// 7.68 kHz.
        Hz7680 = 0b1100,
    }
);

impl DataRate {
    /// Returns the nominal rate in Hz, or `None` for [`DataRate::Off`] and reserved codes.
    pub const fn hz(self) -> Option<f32> {
        Some(match self {
            Self::Hz1_875 => 1.875,
            Self::Hz7_5 => 7.5,
            Self::Hz15 => 15.0,
            Self::Hz30 => 30.0,
            Self::Hz60 => 60.0,
            Self::Hz120 => 120.0,
            Self::Hz240 => 240.0,
            Self::Hz480 => 480.0,
            Self::Hz960 => 960.0,
            Self::Hz1920 => 1920.0,
            Self::Hz3840 => 3840.0,
            Self::Hz7680 => 7680.0,
            Self::Off | Self::Reserved(_) => return None,
        })
    }
}

field_enum!(
    /// Accelerometer operating mode (`OP_MODE_XL`).
    AccelMode, 0b111, {
        /// High-performance mode. The default.
        HighPerformance = 0b000,
        /// High-accuracy ODR mode.
        HighAccuracyOdr = 0b001,
        /// ODR-triggered mode.
        OdrTriggered = 0b011,
        /// Low-power mode 1 (2 samples averaged).
        LowPower1 = 0b100,
        /// Low-power mode 2 (4 samples averaged).
        LowPower2 = 0b101,
        /// Low-power mode 3 (8 samples averaged).
        LowPower3 = 0b110,
        /// Normal mode.
        Normal = 0b111,
    }
);

field_enum!(
    /// Gyroscope operating mode (`OP_MODE_G`).
    GyroMode, 0b111, {
        /// High-performance mode. The default.
        HighPerformance = 0b000,
        /// High-accuracy ODR mode.
        HighAccuracyOdr = 0b001,
        /// ODR-triggered mode.
        OdrTriggered = 0b011,
        /// Sleep mode.
        Sleep = 0b100,
        /// Low-power mode.
        LowPower = 0b101,
    }
);

field_enum!(
    /// Accelerometer full scale (`FS_XL`).
    AccelFullScale, 0b11, {
        /// ±2 g. The default.
        G2 = 0b00,
        /// ±4 g.
        G4 = 0b01,
        /// ±8 g.
        G8 = 0b10,
        /// ±16 g.
        G16 = 0b11,
    }
);

impl AccelFullScale {
    /// Returns the sensitivity in milli-g per LSB.
    pub const fn mg_per_lsb(self) -> f32 {
        match self {
            Self::G2 | Self::Reserved(_) => 0.061,
            Self::G4 => 0.122,
            Self::G8 => 0.244,
            Self::G16 => 0.488,
        }
    }
}

field_enum!(
    /// Gyroscope UI chain full scale (`FS_G`).
    GyroFullScale, 0b1111, {
        /// ±125 dps. The default.
        Dps125 = 0b0000,
        /// ±250 dps.
        Dps250 = 0b0001,
        /// ±500 dps.
        Dps500 = 0b0010,
        /// ±1000 dps.
        Dps1000 = 0b0011,
        /// ±2000 dps.
        Dps2000 = 0b0100,
        /// ±4000 dps. Requires the gyroscope OIS chain to be disabled.
        Dps4000 = 0b1100,
    }
);

impl GyroFullScale {
    /// Returns the sensitivity in milli-degrees per second per LSB.
    pub const fn mdps_per_lsb(self) -> f32 {
        match self {
            Self::Dps125 | Self::Reserved(_) => 4.375,
            Self::Dps250 => 8.75,
            Self::Dps500 => 17.5,
            Self::Dps1000 => 35.0,
            Self::Dps2000 => 70.0,
            Self::Dps4000 => 140.0,
        }
    }
}

field_enum!(
    /// FIFO mode (`FIFO_MODE`).
    FifoMode, 0b111, {
        /// Bypass mode: FIFO disabled. The default.
        Bypass = 0b000,
        /// FIFO mode: stops collecting data when the FIFO is full.
        Fifo = 0b001,
        /// Continuous mode with watermark size until the trigger is deasserted, then FIFO mode.
        ContinuousWtmToFull = 0b010,
        /// Continuous mode until the trigger is deasserted, then FIFO mode.
        ContinuousToFifo = 0b011,
        /// Bypass mode until the trigger is deasserted, then continuous mode.
        BypassToContinuous = 0b100,
        /// Continuous mode: new samples overwrite the oldest when the FIFO is full.
        Continuous = 0b110,
        /// Bypass mode until the trigger is deasserted, then FIFO mode.
        BypassToFifo = 0b111,
    }
);

field_enum!(
    /// Temperature batch data rate in the FIFO (`ODR_T_BATCH`).
    TemperatureBatchRate, 0b11, {
        /// Temperature not batched. The default.
        Off = 0b00,
        /// 1.875 Hz.
        Hz1_875 = 0b01,
        /// 15 Hz.
        Hz15 = 0b10,
        /// 60 Hz.
        Hz60 = 0b11,
    }
);

field_enum!(
    /// Timestamp batching decimation in the FIFO (`DEC_TS_BATCH`), relative to the
    /// highest of the accelerometer and gyroscope batch data rates.
    TimestampDecimation, 0b11, {
        /// Timestamp not batched. The default.
        Off = 0b00,
        /// Every batch.
        X1 = 0b01,
        /// Every 8th batch.
        X8 = 0b10,
        /// Every 32nd batch.
        X32 = 0b11,
    }
);

field_enum!(
    /// Sensor tag of a FIFO word (`TAG_SENSOR`).
    FifoTag, 0b1_1111, {
        /// FIFO empty.
        Empty = 0x00,
        /// Gyroscope, not compressed.
        GyroNc = 0x01,
        /// Accelerometer, not compressed.
        AccelNc = 0x02,
        /// Temperature.
        Temperature = 0x03,
        /// Timestamp.
        Timestamp = 0x04,
        /// Configuration change.
        CfgChange = 0x05,
        /// Accelerometer, not compressed, sample at time T-2.
        AccelNcT2 = 0x06,
        /// Accelerometer, not compressed, sample at time T-1.
        AccelNcT1 = 0x07,
        /// Accelerometer, 2x compressed.
        Accel2xC = 0x08,
        /// Accelerometer, 3x compressed.
        Accel3xC = 0x09,
        /// Gyroscope, not compressed, sample at time T-2.
        GyroNcT2 = 0x0A,
        /// Gyroscope, not compressed, sample at time T-1.
        GyroNcT1 = 0x0B,
        /// Gyroscope, 2x compressed.
        Gyro2xC = 0x0C,
        /// Gyroscope, 3x compressed.
        Gyro3xC = 0x0D,
        /// Sensor hub slave 0.
        SensorHubSlave0 = 0x0E,
        /// Sensor hub slave 1.
        SensorHubSlave1 = 0x0F,
        /// Sensor hub slave 2.
        SensorHubSlave2 = 0x10,
        /// Sensor hub slave 3.
        SensorHubSlave3 = 0x11,
        /// Step counter.
        StepCounter = 0x12,
        /// SFLP game rotation vector.
        SflpGameRotationVector = 0x13,
        /// SFLP gyroscope bias.
        SflpGyroBias = 0x16,
        /// SFLP gravity vector.
        SflpGravityVector = 0x17,
        /// Sensor hub NACK.
        SensorHubNack = 0x19,
        /// Machine learning core result (LSM6DSV16X only).
        MlcResult = 0x1A,
        /// Machine learning core filter (LSM6DSV16X only).
        MlcFilter = 0x1B,
        /// Machine learning core feature (LSM6DSV16X only).
        MlcFeature = 0x1C,
        /// Accelerometer dual-channel.
        AccelDualC = 0x1D,
        /// Enhanced EIS gyroscope.
        GyroEis = 0x1E,
    }
);

field_enum!(
    /// Output data rate of the SFLP game rotation algorithm (`SFLP_GAME_ODR`).
    SflpDataRate, 0b111, {
        /// 15 Hz.
        Hz15 = 0b000,
        /// 30 Hz.
        Hz30 = 0b001,
        /// 60 Hz.
        Hz60 = 0b010,
        /// 120 Hz. The default.
        Hz120 = 0b011,
        /// 240 Hz.
        Hz240 = 0b100,
        /// 480 Hz.
        Hz480 = 0b101,
    }
);

field_enum!(
    /// Accelerometer LPF2 or high-pass filter cutoff (`HP_LPF2_XL_BW`), as a fraction of the ODR.
    ///
    /// With the high-pass path, [`AccelBandwidth::Div4`] selects the slope filter.
    AccelBandwidth, 0b111, {
        /// ODR/4 (slope filter on the high-pass path). The default.
        Div4 = 0b000,
        /// ODR/10.
        Div10 = 0b001,
        /// ODR/20.
        Div20 = 0b010,
        /// ODR/45.
        Div45 = 0b011,
        /// ODR/100.
        Div100 = 0b100,
        /// ODR/200.
        Div200 = 0b101,
        /// ODR/400.
        Div400 = 0b110,
        /// ODR/800.
        Div800 = 0b111,
    }
);

field_enum!(
    /// Self-test mode of the accelerometer (`ST_XL`) or gyroscope (`ST_G`).
    SelfTestMode, 0b11, {
        /// Normal mode. The default.
        Normal = 0b00,
        /// Positive sign self-test.
        Positive = 0b01,
        /// Negative sign self-test.
        Negative = 0b10,
    }
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_code_round_trips() {
        for bits in 0..=0x1F_u8 {
            assert_eq!(DataRate::from_bits(bits).into_bits(), bits & 0xF);
            assert_eq!(AccelMode::from_bits(bits).into_bits(), bits & 0x7);
            assert_eq!(GyroMode::from_bits(bits).into_bits(), bits & 0x7);
            assert_eq!(AccelFullScale::from_bits(bits).into_bits(), bits & 0x3);
            assert_eq!(GyroFullScale::from_bits(bits).into_bits(), bits & 0xF);
            assert_eq!(FifoMode::from_bits(bits).into_bits(), bits & 0x7);
            assert_eq!(
                TemperatureBatchRate::from_bits(bits).into_bits(),
                bits & 0x3
            );
            assert_eq!(TimestampDecimation::from_bits(bits).into_bits(), bits & 0x3);
            assert_eq!(FifoTag::from_bits(bits).into_bits(), bits);
            assert_eq!(SflpDataRate::from_bits(bits).into_bits(), bits & 0x7);
            assert_eq!(AccelBandwidth::from_bits(bits).into_bits(), bits & 0x7);
            assert_eq!(SelfTestMode::from_bits(bits).into_bits(), bits & 0x3);
        }
    }

    #[test]
    fn reserved_codes_are_kept() {
        assert_eq!(
            GyroFullScale::from_bits(0b0101),
            GyroFullScale::Reserved(0b0101)
        );
        assert_eq!(GyroFullScale::from_bits(0b1100), GyroFullScale::Dps4000);
        assert_eq!(DataRate::Hz7680.hz(), Some(7680.0));
        assert_eq!(DataRate::Off.hz(), None);
    }
}
