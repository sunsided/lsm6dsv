//! Filter configuration values.

use lsm6dsv_registers::AccelBandwidth;

/// The accelerometer filter after LPF1 (`CTRL8`, `CTRL9`).
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelFilter {
    /// LPF1 only: bandwidth ODR/2 in high-performance, high-accuracy ODR and normal mode.
    Off,
    /// LPF2 low-pass filter with the specified cutoff.
    LowPass(AccelBandwidth),
    /// High-pass filter with the specified cutoff; [`AccelBandwidth::Div4`] is the slope filter.
    HighPass(AccelBandwidth),
}

impl AccelFilter {
    /// Returns `(LPF2_XL_EN, HP_SLOPE_XL_EN, HP_LPF2_XL_BW)`.
    pub(crate) fn bits(self) -> (bool, bool, AccelBandwidth) {
        match self {
            Self::Off => (false, false, AccelBandwidth::Div4),
            Self::LowPass(bandwidth) => (true, false, bandwidth),
            Self::HighPass(bandwidth) => (false, true, bandwidth),
        }
    }
}
