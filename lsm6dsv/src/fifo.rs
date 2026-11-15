//! FIFO status and tagged FIFO words.

use crate::data::{
    Acceleration, AngularRate, Quaternion, SFLP_GRAVITY_MG_PER_LSB, Temperature, Timestamp, i16x3,
};
use lsm6dsv_registers::{AccelFullScale, FifoDataOutTag, FifoStatus2, FifoTag, GyroFullScale};

/// The FIFO fill level and flags, from `FIFO_STATUS1` and `FIFO_STATUS2`.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoStatus {
    /// The number of unread words (tag plus six data bytes).
    pub level: u16,
    /// The level is at or above the watermark.
    pub watermark: bool,
    /// The FIFO is completely filled.
    pub overrun: bool,
    /// The FIFO will be full at the next ODR.
    pub full: bool,
    /// The BDR counter reached its threshold. Cleared by reading.
    pub counter_bdr: bool,
    /// Latched overrun. Cleared by reading.
    pub overrun_latched: bool,
}

impl FifoStatus {
    /// Decodes `FIFO_STATUS1` and `FIFO_STATUS2`.
    pub fn from_bytes(bytes: [u8; 2]) -> Self {
        let status2 = FifoStatus2::from(bytes[1]);
        Self {
            level: u16::from(bytes[0]) | u16::from(status2.diff_fifo()) << 8,
            watermark: status2.fifo_wtm_ia(),
            overrun: status2.fifo_ovr_ia(),
            full: status2.fifo_full_ia(),
            counter_bdr: status2.counter_bdr_ia(),
            overrun_latched: status2.fifo_ovr_latched(),
        }
    }
}

/// A raw FIFO word: the tag byte plus six data bytes.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoWord {
    /// The sensor that produced the word.
    pub tag: FifoTag,
    /// The 2-bit time slot counter (`TAG_CNT`).
    pub count: u8,
    /// The data bytes (`FIFO_DATA_OUT_X_L` to `FIFO_DATA_OUT_Z_H`).
    pub data: [u8; 6],
}

impl FifoWord {
    /// Decodes the seven bytes starting at `FIFO_DATA_OUT_TAG`.
    pub fn from_bytes(bytes: &[u8; 7]) -> Self {
        let tag = FifoDataOutTag::from(bytes[0]);
        let mut data = [0; 6];
        data.copy_from_slice(&bytes[1..]);
        Self {
            tag: tag.tag_sensor(),
            count: tag.tag_cnt(),
            data,
        }
    }

    /// Decodes the word, scaling accelerometer and gyroscope data with the specified full scales.
    ///
    /// The full scales must be the ones the samples were taken at. A [`FifoTag::CfgChange`]
    /// word marks a configuration change.
    pub fn decode(&self, accel: AccelFullScale, gyro: GyroFullScale) -> FifoSample {
        let d = &self.data;
        match self.tag {
            FifoTag::AccelNc | FifoTag::AccelDualC => {
                FifoSample::Acceleration(Acceleration::from_bytes(d, accel))
            }
            FifoTag::GyroNc | FifoTag::GyroEis => {
                FifoSample::AngularRate(AngularRate::from_bytes(d, gyro))
            }
            FifoTag::Temperature => FifoSample::Temperature(Temperature::from_bytes([d[0], d[1]])),
            FifoTag::Timestamp => {
                FifoSample::Timestamp(Timestamp::from_bytes([d[0], d[1], d[2], d[3]]))
            }
            FifoTag::StepCounter => FifoSample::StepCounter {
                steps: u16::from_le_bytes([d[0], d[1]]),
                timestamp: Timestamp::from_bytes([d[2], d[3], d[4], d[5]]),
            },
            FifoTag::SflpGameRotationVector => {
                FifoSample::GameRotation(Quaternion::from_fifo_bytes(d))
            }
            FifoTag::SflpGravityVector => {
                FifoSample::Gravity(i16x3(d).map(|v| f32::from(v) * SFLP_GRAVITY_MG_PER_LSB))
            }
            FifoTag::SflpGyroBias => FifoSample::GyroBias(i16x3(d)),
            FifoTag::SensorHubSlave0 => FifoSample::SensorHub { slave: 0, data: *d },
            FifoTag::SensorHubSlave1 => FifoSample::SensorHub { slave: 1, data: *d },
            FifoTag::SensorHubSlave2 => FifoSample::SensorHub { slave: 2, data: *d },
            FifoTag::SensorHubSlave3 => FifoSample::SensorHub { slave: 3, data: *d },
            FifoTag::Empty => FifoSample::Empty,
            _ => FifoSample::Other(*self),
        }
    }
}

/// A decoded FIFO word.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoSample {
    /// The FIFO was empty.
    Empty,
    /// An uncompressed accelerometer sample (also dual-channel).
    Acceleration(Acceleration),
    /// An uncompressed gyroscope sample (also enhanced EIS).
    AngularRate(AngularRate),
    /// A temperature sample.
    Temperature(Temperature),
    /// A timestamp.
    Timestamp(Timestamp),
    /// A step counter value with its timestamp.
    StepCounter {
        /// The step count.
        steps: u16,
        /// The time of the last step.
        timestamp: Timestamp,
    },
    /// The SFLP game rotation vector.
    GameRotation(Quaternion),
    /// The SFLP gravity vector, in mg.
    Gravity([f32; 3]),
    /// The raw SFLP gyroscope bias.
    GyroBias([i16; 3]),
    /// Sensor hub data of the specified slave.
    SensorHub {
        /// The slave index, 0 to 3.
        slave: u8,
        /// The data bytes.
        data: [u8; 6],
    },
    /// A word the decoder does not interpret: compressed data, configuration changes,
    /// sensor hub NACKs and machine learning core output.
    Other(FifoWord),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status() {
        let status = FifoStatus::from_bytes([0x2C, 0b1000_0001]);
        assert_eq!(status.level, 0x12C);
        assert!(status.watermark);
        assert!(!status.overrun);
    }

    #[test]
    fn decode_words() {
        let word = FifoWord::from_bytes(&[0x02 << 3 | 0b01 << 1, 0x10, 0, 0, 0, 0xF0, 0xFF]);
        assert_eq!(word.tag, FifoTag::AccelNc);
        assert_eq!(word.count, 1);
        let FifoSample::Acceleration(a) = word.decode(AccelFullScale::G2, GyroFullScale::Dps125)
        else {
            panic!("not an acceleration");
        };
        assert_eq!(a.raw, [16, 0, -16]);

        let steps = FifoWord::from_bytes(&[0x12 << 3, 7, 0, 1, 0, 0, 0]);
        assert_eq!(
            steps.decode(AccelFullScale::G2, GyroFullScale::Dps125),
            FifoSample::StepCounter {
                steps: 7,
                timestamp: Timestamp { raw: 1 }
            }
        );

        let compressed = FifoWord::from_bytes(&[0x08 << 3, 0, 0, 0, 0, 0, 0]);
        assert!(matches!(
            compressed.decode(AccelFullScale::G2, GyroFullScale::Dps125),
            FifoSample::Other(_)
        ));
    }
}
