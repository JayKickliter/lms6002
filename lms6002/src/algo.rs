use self::FRange::*;
use self::SelVco::*;
use error::*;
use reg::{FRange, SelVco};

#[derive(Debug)]
pub struct TuningParams {
    pub selvco: SelVco,
    pub frange: FRange,
    pub nint: u16,
    pub nfrac: u32,
}

const FREQSEL_LUT: [(u32, u32, (SelVco, FRange)); 16] = [
    (232_500_000, 285_625_000, (Vco4, VcoDiv16)),
    (285_625_000, 336_875_000, (Vco3, VcoDiv16)),
    (336_875_000, 405_000_000, (Vco2, VcoDiv16)),
    (405_000_000, 465_000_000, (Vco1, VcoDiv16)),
    (465_000_000, 571_250_000, (Vco4, VcoDiv8)),
    (571_250_000, 673_750_000, (Vco3, VcoDiv8)),
    (673_750_000, 810_000_000, (Vco2, VcoDiv8)),
    (810_000_000, 930_000_000, (Vco1, VcoDiv8)),
    (930_000_000, 1_142_500_000, (Vco4, VcoDiv4)),
    (1_142_500_000, 1_347_500_000, (Vco3, VcoDiv4)),
    (1_347_500_000, 1_620_000_000, (Vco2, VcoDiv4)),
    (1_620_000_000, 1_860_000_000, (Vco1, VcoDiv4)),
    (1_860_000_000, 2_285_000_000, (Vco4, VcoDiv2)),
    (2_285_000_000, 2_695_000_000, (Vco3, VcoDiv2)),
    (2_695_000_000, 3_240_000_000, (Vco2, VcoDiv2)),
    (3_240_000_000, 3_720_000_000, (Vco1, VcoDiv2)),
];

fn freqsel(freq: u32) -> Result<(SelVco, FRange)> {
    FREQSEL_LUT
        .iter()
        .find(|&&(l, h, _)| l < freq && freq < h)
        .map(|&(_, _, val)| val)
        .ok_or(Error::Range)
}

#[cfg(test)]
#[test]
fn test_freqsel() {
    for &(l, h, val) in &FREQSEL_LUT {
        let freq = (h - l) / 2 + l;
        assert_eq!(freqsel(freq), Ok(val));
        let freq = l - 1;
        assert!(freqsel(freq) != Ok(val));
        let freq = h + 1;
        assert!(freqsel(freq) != Ok(val));
    }
}

pub fn params_to_freq(refclk: u32, f: &TuningParams) -> f64 {
    let x = 2u32.pow((u32::from(u8::from(f.frange))) - 3);
    (f64::from(f.nint) + f64::from(f.nfrac) / f64::from(1 << 23))
        * (f64::from(refclk) / f64::from(x))
}

pub fn params_from_freq(refclk: u32, freq: f64) -> Result<TuningParams> {
    // For wanted LO frequency f_lo and given PLL reference clock
    // frequency f_ref, calculate integer and fractional part of the
    // divider as below.

    // First, find temporary variable x from the 3 least significant
    // bits of the FREQSEL value:
    let (selvco, frange) = freqsel(freq as u32)?;
    let x = 2u32.pow((u32::from(u8::from(frange))) - 3);
    // Use x to calculate NINT and NFRAC:
    let nint = ((u64::from(x) * freq as u64) / u64::from(refclk)) as u32;
    let nfrac = 2f64.powi(23) * ((f64::from(x) * freq) / f64::from(refclk) - f64::from(nint));
    Ok(TuningParams {
        selvco,
        frange,
        nint: nint as u16,
        nfrac: nfrac as u32,
    })
}

#[cfg(test)]
#[test]
fn test_freq_to_params() {
    // The following is from an example from section 3.4.2 of
    // "LMS6002 – Wide Band Multi Standard Radio Chip - Programming and Calibration Guide"
    const REFCLK: u32 = 30_720_000;
    let freq = 2_140_000_000.0;
    let p = params_from_freq(REFCLK, freq).unwrap();
    assert_eq!(p.selvco, SelVco::from(0b100));
    assert_eq!(p.frange, FRange::from(0b100));
    assert_eq!(p.nint, 139);
    assert_eq!(p.nfrac, 2708821);
}

const LPF_LUT: [(f64, u8); 16] = [
    (14e6, 0b0000),
    (10e6, 0b0001),
    (7e6, 0b0010),
    (6e6, 0b0011),
    (5e6, 0b0100),
    (4.375e6, 0b0101),
    (3.5e6, 0b0110),
    (3e6, 0b0111),
    (2.75e6, 0b1000),
    (2.5e6, 0b1001),
    (1.92e6, 0b1010),
    (1.5e6, 0b1011),
    (1.375e6, 0b1100),
    (1.25e6, 0b1101),
    (0.875e6, 0b1110),
    (0.75e6, 0b1111),
];

pub fn bwc_from_bw(freq: f64) -> Result<u8> {
    LPF_LUT
        .iter()
        .find(|&&(f, _)| f <= freq)
        .map(|&(_, val)| val)
        .ok_or(Error::Range)
}

pub fn bw_from_bwc(bwc: u8) -> Result<f64> {
    LPF_LUT
        .iter()
        .find(|&&(_, b)| b == bwc)
        .map(|&(f, _)| f)
        .ok_or(Error::Range)
}

#[cfg(test)]
#[test]
fn test_bwc_from_bw() {
    for (freq, regval) in &LPF_LUT {
        assert_eq!(Ok(*regval), bwc_from_bw(*freq));
    }
}

pub fn rxga2_gain_from_field(field: u8) -> Result<u32> {
    debug_assert!(field <= 0b10100);
    if field > 0b10100 {
        Err(Error::Range)
    } else {
        Ok(u32::from(field) * 3)
    }
}

#[cfg(test)]
#[test]
fn test_rxga2_gain_from_field() {
    assert_eq!(rxga2_gain_from_field(0b00000), Ok(0));
    assert_eq!(rxga2_gain_from_field(0b00001), Ok(3));
    assert_eq!(rxga2_gain_from_field(0b01001), Ok(27));
    assert_eq!(rxga2_gain_from_field(0b01010), Ok(30));
    assert_eq!(rxga2_gain_from_field(0b10100), Ok(60));
}

pub fn rxga2_gain_to_field(gain: u32) -> Result<u8> {
    if gain <= 60 && gain % 3 == 0 {
        Ok((gain / 3) as u8)
    } else {
        Err(Error::Range)
    }
}

#[cfg(test)]
#[test]
fn test_rxga2_gain_to_field() {
    assert_eq!(rxga2_gain_to_field(0), Ok(0b00000));
    assert_eq!(rxga2_gain_to_field(3), Ok(0b00001));
    assert_eq!(rxga2_gain_to_field(27), Ok(0b01001));
    assert_eq!(rxga2_gain_to_field(30), Ok(0b01010));
    assert_eq!(rxga2_gain_to_field(60), Ok(0b10100));
}
