//! Measurement values and their decoding from register bytes.

use lsm6dsv_registers::{
    AccelFullScale, GyroFullScale, TEMPERATURE_LSB_PER_CELSIUS, TEMPERATURE_OFFSET_CELSIUS,
    TIMESTAMP_MICROSECONDS_PER_LSB,
};

/// Standard gravity, in m/s².
pub const STANDARD_GRAVITY: f32 = 9.806_65;

/// Qvar gain, in LSB per mV (datasheet table 7, typical).
pub const QVAR_LSB_PER_MILLIVOLT: f32 = 78.0;

/// Sensitivity of the SFLP gravity vector, in mg per LSB.
pub const SFLP_GRAVITY_MG_PER_LSB: f32 = 0.061;

/// Decodes three little-endian `i16` values.
pub(crate) const fn i16x3(bytes: &[u8; 6]) -> [i16; 3] {
    [
        i16::from_le_bytes([bytes[0], bytes[1]]),
        i16::from_le_bytes([bytes[2], bytes[3]]),
        i16::from_le_bytes([bytes[4], bytes[5]]),
    ]
}

/// An accelerometer sample.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Acceleration {
    /// The raw X, Y and Z outputs.
    pub raw: [i16; 3],
    /// The full scale the sample was taken at.
    pub scale: AccelFullScale,
}

impl Acceleration {
    /// Decodes the six output bytes (X, Y, Z, little-endian).
    pub const fn from_bytes(bytes: &[u8; 6], scale: AccelFullScale) -> Self {
        Self {
            raw: i16x3(bytes),
            scale,
        }
    }

    /// Returns the X, Y and Z acceleration in milli-g.
    pub fn milli_g(&self) -> [f32; 3] {
        self.raw.map(|v| f32::from(v) * self.scale.mg_per_lsb())
    }

    /// Returns the X, Y and Z acceleration in g.
    pub fn g(&self) -> [f32; 3] {
        self.milli_g().map(|v| v / 1000.0)
    }

    /// Returns the X, Y and Z acceleration in m/s².
    pub fn meters_per_second_squared(&self) -> [f32; 3] {
        self.g().map(|v| v * STANDARD_GRAVITY)
    }
}

/// A gyroscope sample.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AngularRate {
    /// The raw X, Y and Z outputs.
    pub raw: [i16; 3],
    /// The full scale the sample was taken at.
    pub scale: GyroFullScale,
}

impl AngularRate {
    /// Decodes the six output bytes (X, Y, Z, little-endian).
    pub const fn from_bytes(bytes: &[u8; 6], scale: GyroFullScale) -> Self {
        Self {
            raw: i16x3(bytes),
            scale,
        }
    }

    /// Returns the X, Y and Z angular rate in degrees per second.
    pub fn degrees_per_second(&self) -> [f32; 3] {
        self.raw
            .map(|v| f32::from(v) * self.scale.mdps_per_lsb() / 1000.0)
    }

    /// Returns the X, Y and Z angular rate in radians per second.
    pub fn radians_per_second(&self) -> [f32; 3] {
        self.degrees_per_second()
            .map(|v| v * core::f32::consts::PI / 180.0)
    }
}

/// A temperature sample.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Temperature {
    /// The raw output.
    pub raw: i16,
}

impl Temperature {
    /// Decodes the two output bytes (little-endian).
    pub const fn from_bytes(bytes: [u8; 2]) -> Self {
        Self {
            raw: i16::from_le_bytes(bytes),
        }
    }

    /// Returns the temperature in °C.
    pub fn celsius(&self) -> f32 {
        f32::from(self.raw) / TEMPERATURE_LSB_PER_CELSIUS + TEMPERATURE_OFFSET_CELSIUS
    }
}

/// A timestamp counter value.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timestamp {
    /// The raw counter value (21.75 µs per LSB, typical).
    pub raw: u32,
}

impl Timestamp {
    /// Decodes the four counter bytes (little-endian).
    pub const fn from_bytes(bytes: [u8; 4]) -> Self {
        Self {
            raw: u32::from_le_bytes(bytes),
        }
    }

    /// Returns the time in microseconds, based on the typical resolution.
    ///
    /// The actual resolution varies per device; `INTERNAL_FREQ_FINE` holds the deviation.
    pub fn microseconds(&self) -> f64 {
        f64::from(self.raw) * f64::from(TIMESTAMP_MICROSECONDS_PER_LSB)
    }
}

/// A Qvar (electrostatic charge variation) sample (LSM6DSV16X).
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Qvar {
    /// The raw output.
    pub raw: i16,
}

impl Qvar {
    /// Returns the voltage in mV, based on the typical gain.
    pub fn millivolts(&self) -> f32 {
        f32::from(self.raw) / QVAR_LSB_PER_MILLIVOLT
    }
}

/// A unit quaternion from the SFLP game rotation vector.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Quaternion {
    /// The X component.
    pub x: f32,
    /// The Y component.
    pub y: f32,
    /// The Z component.
    pub z: f32,
    /// The scalar component, derived from X, Y and Z (the device stores only those).
    pub w: f32,
}

impl Quaternion {
    /// Decodes the X, Y and Z half-precision components of a FIFO word.
    pub fn from_fifo_bytes(bytes: &[u8; 6]) -> Self {
        let [x, y, z] = [0, 2, 4].map(|i| f16_to_f32(u16::from_le_bytes([bytes[i], bytes[i + 1]])));
        let sum = x * x + y * y + z * z;
        // Rounding can push the sum slightly above 1.
        let w = if sum < 1.0 { sqrt(1.0 - sum) } else { 0.0 };
        Self { x, y, z, w }
    }
}

/// Converts an IEEE 754 half-precision value to `f32`.
pub fn f16_to_f32(half: u16) -> f32 {
    let sign = u32::from(half >> 15) << 31;
    let exponent = u32::from((half >> 10) & 0x1F);
    let mantissa = u32::from(half & 0x3FF);
    let bits = match (exponent, mantissa) {
        (0, 0) => sign,
        // Subnormal: value = mantissa × 2^-24.
        (0, m) => {
            // 2^-24 is the smallest half-precision subnormal.
            let magnitude = m as f32 * f32::from_bits(0x3380_0000);
            return if sign != 0 { -magnitude } else { magnitude };
        }
        (0x1F, m) => sign | 0x7F80_0000 | (m << 13),
        (e, m) => sign | ((e + 127 - 15) << 23) | (m << 13),
    };
    f32::from_bits(bits)
}

/// Converts an `f32` to IEEE 754 half precision, rounding to nearest.
pub fn f32_to_f16(value: f32) -> u16 {
    let bits = value.to_bits();
    let sign = ((bits >> 16) & 0x8000) as u16;
    let exponent = ((bits >> 23) & 0xFF) as i32;
    let mantissa = bits & 0x7F_FFFF;
    if exponent == 0xFF {
        return sign | 0x7C00 | if mantissa != 0 { 0x200 } else { 0 };
    }
    let half_exponent = exponent - 127 + 15;
    if half_exponent >= 0x1F {
        return sign | 0x7C00;
    }
    if half_exponent <= 0 {
        if half_exponent < -10 {
            return sign;
        }
        let m = (mantissa | 0x80_0000) >> (1 - half_exponent);
        return sign | ((m + 0x1000) >> 13) as u16;
    }
    let half = sign as u32 | ((half_exponent as u32) << 10) | (mantissa >> 13);
    // Round to nearest; a carry into the exponent is correct.
    (half + ((mantissa >> 12) & 1)) as u16
}

/// Newton's method square root, since `no_std` has no `f32::sqrt`.
fn sqrt(value: f32) -> f32 {
    if value <= 0.0 {
        return 0.0;
    }
    let mut x = if value > 1.0 { value } else { 1.0 };
    for _ in 0..20 {
        x = 0.5 * (x + value / x);
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scales() {
        let a = Acceleration::from_bytes(&[0xE8, 0x03, 0, 0, 0x18, 0xFC], AccelFullScale::G4);
        assert_eq!(a.raw, [1000, 0, -1000]);
        assert_eq!(a.milli_g(), [122.0, 0.0, -122.0]);
        let g = AngularRate::from_bytes(&[100, 0, 0, 0, 0, 0], GyroFullScale::Dps2000);
        assert_eq!(g.degrees_per_second()[0], 7.0);
        assert_eq!(Temperature::from_bytes([0, 1]).celsius(), 26.0);
        assert_eq!(Timestamp::from_bytes([4, 0, 0, 0]).microseconds(), 87.0);
        assert_eq!(Qvar { raw: 156 }.millivolts(), 2.0);
    }

    #[test]
    fn half_precision() {
        assert_eq!(f16_to_f32(0x3C00), 1.0);
        assert_eq!(f16_to_f32(0xB800), -0.5);
        assert_eq!(f16_to_f32(0x0000), 0.0);
        assert_eq!(f16_to_f32(0x0001), 5.960_464_5e-8);
        assert_eq!(f16_to_f32(0x8001), -5.960_464_5e-8);
        assert!(f16_to_f32(0x7C00).is_infinite());
        for half in [0x3C00_u16, 0xB800, 0x3555, 0x0400, 0x7BFF, 0x8000, 0x0001] {
            assert_eq!(f32_to_f16(f16_to_f32(half)), half, "{half:#06x}");
        }
        assert_eq!(f32_to_f16(1.0e9), 0x7C00);
    }

    #[test]
    fn quaternion_w() {
        // x = 0.5, y = 0.5, z = 0.5 → w = 0.5.
        let q = Quaternion::from_fifo_bytes(&[0x00, 0x38, 0x00, 0x38, 0x00, 0x38]);
        assert!((q.w - 0.5).abs() < 1e-6);
    }
}
