use error::*;

#[derive(Debug)]
pub struct TuningParams {
    pub selvco: u8,
    pub frange: u8,
    pub nint: u16,
    pub nfrac: u32,
}

const FREQSEL_LUT: [(u32, u32, (u8, u8)); 16] = [
    (232_500_000, 285_625_000, (0b100, 0b111)),
    (285_625_000, 336_875_000, (0b101, 0b111)),
    (336_875_000, 405_000_000, (0b110, 0b111)),
    (405_000_000, 465_000_000, (0b111, 0b111)),
    (465_000_000, 571_250_000, (0b100, 0b110)),
    (571_250_000, 673_750_000, (0b101, 0b110)),
    (673_750_000, 810_000_000, (0b110, 0b110)),
    (810_000_000, 930_000_000, (0b111, 0b110)),
    (930_000_000, 1_142_500_000, (0b100, 0b101)),
    (1_142_500_000, 1_347_500_000, (0b101, 0b101)),
    (1_347_500_000, 1_620_000_000, (0b110, 0b101)),
    (1_620_000_000, 1_860_000_000, (0b111, 0b101)),
    (1_860_000_000, 2_285_000_000, (0b100, 0b100)),
    (2_285_000_000, 2_695_000_000, (0b101, 0b100)),
    (2_695_000_000, 3_240_000_000, (0b110, 0b100)),
    (3_240_000_000, 3_720_000_000, (0b111, 0b100)),
];

fn freqsel(freq: u32) -> Result<(u8, u8)> {
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

pub fn freq_to_params(refclk: u32, freq: f64) -> Result<TuningParams> {
    // For wanted LO frequency f_lo and given PLL reference clock
    // frequency f_ref, calculate integer and fractional part of the
    // divider as below.

    // First, find temporary variable x from the 3 least significant
    // bits of the FREQSEL value:
    let (selvco, frange) = freqsel(freq as u32)?;
    let x = 2u32.pow((u32::from(frange) & 0b111) - 3);
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
    let TuningParams {
        selvco,
        frange,
        nint,
        nfrac,
    } = freq_to_params(REFCLK, freq).unwrap();
    assert_eq!(selvco, 0b100);
    assert_eq!(frange, 0b100);
    assert_eq!(nint, 139);
    assert_eq!(nfrac, 2708821);
}
