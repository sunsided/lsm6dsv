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
    /// word marks a configuration change. Compressed words need the previous sample; decode
    /// them with a [`FifoDecoder`].
    pub fn decode(&self, accel: AccelFullScale, gyro: GyroFullScale) -> FifoSample {
        let d = &self.data;
        match self.tag {
            FifoTag::AccelNc | FifoTag::AccelNcT1 | FifoTag::AccelNcT2 | FifoTag::AccelDualC => {
                FifoSample::Acceleration(Acceleration::from_bytes(d, accel))
            }

            FifoTag::GyroNc | FifoTag::GyroNcT1 | FifoTag::GyroNcT2 | FifoTag::GyroEis => {
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
    /// A word the decoder does not interpret: compressed data (without a [`FifoDecoder`]),
    /// configuration changes, sensor hub NACKs and machine learning core output.
    Other(FifoWord),
}

/// Decodes a stream of FIFO words, including compressed accelerometer and gyroscope data.
///
/// With compression enabled, the device stores most samples as differences to the previous
/// sample of the same sensor: a 2x compressed word holds two samples (8-bit differences), a
/// 3x compressed word three (5-bit differences). The decoder keeps the last sample of each
/// sensor to restore them. Feed it every word in FIFO order, starting with an uncompressed one
/// (the device forces one at the rate set by `FIFO_CTRL2.UNCOMPR_RATE`).
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoDecoder {
    accel_scale: AccelFullScale,
    gyro_scale: GyroFullScale,
    last_accel: [i16; 3],
    last_gyro: [i16; 3],
}

impl FifoDecoder {
    /// Creates a decoder for samples taken at the specified full scales.
    pub const fn new(accel_scale: AccelFullScale, gyro_scale: GyroFullScale) -> Self {
        Self {
            accel_scale,
            gyro_scale,
            last_accel: [0; 3],
            last_gyro: [0; 3],
        }
    }

    /// Changes the full scales, for example after a [`FifoTag::CfgChange`] word.
    pub fn set_scales(&mut self, accel_scale: AccelFullScale, gyro_scale: GyroFullScale) {
        self.accel_scale = accel_scale;
        self.gyro_scale = gyro_scale;
    }

    /// Decodes a word into up to three samples, oldest first.
    pub fn decode(&mut self, word: &FifoWord) -> DecodedSamples {
        let mut out = DecodedSamples::default();
        let accel = matches!(
            word.tag,
            FifoTag::AccelNc
                | FifoTag::AccelNcT1
                | FifoTag::AccelNcT2
                | FifoTag::Accel2xC
                | FifoTag::Accel3xC
        );

        match word.tag {
            FifoTag::Accel2xC | FifoTag::Gyro2xC => {
                let differences = word.data.map(|b| i16::from(b as i8));
                self.restore(accel, &differences, &mut out);
            }

            FifoTag::Accel3xC | FifoTag::Gyro3xC => {
                self.restore(accel, &differences_3x(&word.data), &mut out);
            }

            FifoTag::AccelNc | FifoTag::AccelNcT1 | FifoTag::AccelNcT2 => {
                self.last_accel = i16x3(&word.data);
                out.push(word.decode(self.accel_scale, self.gyro_scale));
            }

            FifoTag::GyroNc | FifoTag::GyroNcT1 | FifoTag::GyroNcT2 => {
                self.last_gyro = i16x3(&word.data);
                out.push(word.decode(self.accel_scale, self.gyro_scale));
            }

            _ => out.push(word.decode(self.accel_scale, self.gyro_scale)),
        }

        out
    }

    /// Applies consecutive X, Y, Z differences to the last sample of a sensor.
    fn restore(&mut self, accel: bool, differences: &[i16], out: &mut DecodedSamples) {
        for step in differences.chunks_exact(3) {
            let last = if accel {
                &mut self.last_accel
            } else {
                &mut self.last_gyro
            };

            for (value, difference) in last.iter_mut().zip(step) {
                *value = value.wrapping_add(*difference);
            }

            let raw = *last;
            out.push(if accel {
                FifoSample::Acceleration(Acceleration {
                    raw,
                    scale: self.accel_scale,
                })
            } else {
                FifoSample::AngularRate(AngularRate {
                    raw,
                    scale: self.gyro_scale,
                })
            });
        }
    }
}

/// Unpacks the nine 5-bit two's complement differences of a 3x compressed word: three
/// little-endian 16-bit words, each holding X, Y and Z in bits 4:0, 9:5 and 14:10.
fn differences_3x(data: &[u8; 6]) -> [i16; 9] {
    let mut out = [0; 9];

    for sample in 0..3 {
        let word = u16::from_le_bytes([data[2 * sample], data[2 * sample + 1]]);
        for axis in 0..3 {
            let bits = ((word >> (5 * axis)) & 0x1F) as i16;
            out[3 * sample + axis] = if bits < 16 { bits } else { bits - 32 };
        }
    }

    out
}

/// Up to three samples decoded from one FIFO word, oldest first.
#[derive(Debug, Copy, Clone, Default, PartialEq)]
pub struct DecodedSamples {
    samples: [Option<FifoSample>; 3],
    next: usize,
}

impl DecodedSamples {
    fn push(&mut self, sample: FifoSample) {
        if let Some(slot) = self.samples.iter_mut().find(|s| s.is_none()) {
            *slot = Some(sample);
        }
    }
}

impl Iterator for DecodedSamples {
    type Item = FifoSample;

    fn next(&mut self) -> Option<Self::Item> {
        let sample = self.samples.get_mut(self.next)?.take();
        self.next += 1;
        sample
    }
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
        // Without a decoder, compressed words stay undecoded.
        assert!(matches!(
            compressed.decode(AccelFullScale::G2, GyroFullScale::Dps125),
            FifoSample::Other(_)
        ));
    }

    fn raw_samples(samples: DecodedSamples) -> ([[i16; 3]; 3], usize) {
        let mut out = [[0; 3]; 3];
        let mut count = 0;

        for sample in samples {
            out[count] = match sample {
                FifoSample::Acceleration(a) => a.raw,
                FifoSample::AngularRate(g) => g.raw,
                other => panic!("unexpected {other:?}"),
            };
            count += 1;
        }

        (out, count)
    }

    #[test]
    fn decompression() {
        let mut decoder = FifoDecoder::new(AccelFullScale::G2, GyroFullScale::Dps125);

        // Uncompressed accelerometer sample [100, -100, 1000].
        let nc = FifoWord::from_bytes(&[0x02 << 3, 100, 0, 0x9C, 0xFF, 0xE8, 0x03]);
        assert_eq!(raw_samples(decoder.decode(&nc)).1, 1);

        // 2x: +1/-1/+2, then -3/+4/0 (cumulative).
        let two = FifoWord::from_bytes(&[0x08 << 3, 1, 0xFF, 2, 0xFD, 4, 0]);
        let (samples, count) = raw_samples(decoder.decode(&two));
        assert_eq!(count, 2);
        assert_eq!(samples[0], [101, -101, 1002]);
        assert_eq!(samples[1], [98, -97, 1002]);

        // 3x: (1, -1, 15), (-16, 0, 2), (0, 0, -1) as 5-bit fields.
        let pack = |x: i16, y: i16, z: i16| {
            ((x & 0x1F) as u16 | ((y & 0x1F) as u16) << 5 | ((z & 0x1F) as u16) << 10).to_le_bytes()
        };
        let [a, b] = pack(1, -1, 15);
        let [c, d] = pack(-16, 0, 2);
        let [e, f] = pack(0, 0, -1);
        let three = FifoWord::from_bytes(&[0x09 << 3, a, b, c, d, e, f]);
        let (samples, count) = raw_samples(decoder.decode(&three));
        assert_eq!(count, 3);
        assert_eq!(samples[0], [99, -98, 1017]);
        assert_eq!(samples[1], [83, -98, 1019]);
        assert_eq!(samples[2], [83, -98, 1018]);

        // Gyroscope state is separate.
        let gyro = FifoWord::from_bytes(&[0x0C << 3, 5, 0, 0, 0, 0, 0]);
        let (samples, _) = raw_samples(decoder.decode(&gyro));
        assert_eq!(samples[0], [5, 0, 0]);
    }

    #[test]
    fn decoder_passes_other_words_through() {
        let mut decoder = FifoDecoder::new(AccelFullScale::G2, GyroFullScale::Dps125);
        let ts = FifoWord::from_bytes(&[0x04 << 3, 9, 0, 0, 0, 0, 0]);
        let mut samples = decoder.decode(&ts);
        assert_eq!(
            samples.next(),
            Some(FifoSample::Timestamp(Timestamp { raw: 9 }))
        );
        assert_eq!(samples.next(), None);
    }
}
