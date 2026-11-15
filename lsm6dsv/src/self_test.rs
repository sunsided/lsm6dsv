//! Self-test evaluation, following the limits of datasheet DS13510 (table 2).

use crate::{AccelFullScale, GyroFullScale};

/// Minimum accelerometer self-test output change, in mg (full-scale independent).
pub const ACCEL_SELF_TEST_MIN_MG: f32 = 50.0;

/// Maximum accelerometer self-test output change, in mg (full-scale independent).
pub const ACCEL_SELF_TEST_MAX_MG: f32 = 1700.0;

/// Minimum gyroscope self-test output change at ±2000 dps, in dps.
pub const GYRO_SELF_TEST_MIN_DPS: f32 = 150.0;

/// Maximum gyroscope self-test output change at ±2000 dps, in dps.
pub const GYRO_SELF_TEST_MAX_DPS: f32 = 700.0;

/// The number of samples averaged with and without self-test.
pub const SELF_TEST_SAMPLES: i32 = 5;

/// The accelerometer full scale used during the self-test.
pub(crate) const ACCEL_SCALE: AccelFullScale = AccelFullScale::G4;

/// The gyroscope full scale used during the self-test, which the limits refer to.
pub(crate) const GYRO_SCALE: GyroFullScale = GyroFullScale::Dps2000;

/// Settling time after changing the configuration or enabling self-test, in milliseconds.
pub(crate) const SETTLE_MS: u32 = 100;

/// Poll interval while waiting for a sample, in milliseconds.
pub(crate) const POLL_MS: u32 = 1;

/// Maximum number of polls while waiting for a sample.
pub(crate) const MAX_POLLS: u32 = 100;

/// The result of a self-test.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SelfTest {
    /// The absolute output change per axis, in mg (accelerometer) or dps (gyroscope).
    pub change: [f32; 3],
    /// The lower limit of the change.
    pub min: f32,
    /// The upper limit of the change.
    pub max: f32,
}

impl SelfTest {
    /// Whether the change of every axis is within the limits.
    pub fn passed(&self) -> bool {
        self.change
            .iter()
            .all(|&change| (self.min..=self.max).contains(&change))
    }

    /// Evaluates the accelerometer from raw sample sums without and with self-test.
    pub(crate) fn accel(without: [i32; 3], with: [i32; 3]) -> Self {
        Self {
            change: change(without, with, ACCEL_SCALE.mg_per_lsb()),
            min: ACCEL_SELF_TEST_MIN_MG,
            max: ACCEL_SELF_TEST_MAX_MG,
        }
    }

    /// Evaluates the gyroscope from raw sample sums without and with self-test.
    pub(crate) fn gyro(without: [i32; 3], with: [i32; 3]) -> Self {
        Self {
            change: change(without, with, GYRO_SCALE.mdps_per_lsb() / 1000.0),
            min: GYRO_SELF_TEST_MIN_DPS,
            max: GYRO_SELF_TEST_MAX_DPS,
        }
    }
}

/// The absolute difference of the averages, scaled.
fn change(without: [i32; 3], with: [i32; 3], scale: f32) -> [f32; 3] {
    let mut out = [0.0; 3];

    for axis in 0..3 {
        let difference = (with[axis] - without[axis]) as f32 / SELF_TEST_SAMPLES as f32;
        out[axis] = if difference < 0.0 {
            -difference
        } else {
            difference
        } * scale;
    }

    out
}

/// Adds a sample to a sum.
pub(crate) fn accumulate(sum: &mut [i32; 3], raw: [i16; 3]) {
    for (total, value) in sum.iter_mut().zip(raw) {
        *total += i32::from(value);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accel_limits() {
        // 4098 LSB at ±4 g is 500 mg.
        let n = SELF_TEST_SAMPLES;
        let result = SelfTest::accel([0, 0, 1000 * n], [4098 * n, -4098 * n, 1000 * n]);
        assert!((result.change[0] - 499.956).abs() < 0.01);
        assert_eq!(result.change[2], 0.0);
        assert!(!result.passed());

        let result = SelfTest::accel([0; 3], [4098 * n; 3]);
        assert!(result.passed());
    }

    #[test]
    fn gyro_limits() {
        // 4096 LSB at ±2000 dps is 286.72 dps.
        let n = SELF_TEST_SAMPLES;
        assert!(SelfTest::gyro([0; 3], [-4096 * n; 3]).passed());
        assert!(!SelfTest::gyro([0; 3], [1000 * n; 3]).passed());
    }
}
