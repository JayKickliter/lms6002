use error::*;

#[derive(Debug)]
struct TuningParams {
    pub freqsel: u8,
    pub nint: u16,
    pub nfrac: u32,
}

const FREQSEL_LUT: [(u32, u32, u8); 16] = [
    (232_500_000, 285_625_000, 0b10_0111),
    (285_625_000, 336_875_000, 0b10_1111),
    (336_875_000, 405_000_000, 0b11_0111),
    (405_000_000, 465_000_000, 0b11_1111),
    (465_000_000, 571_250_000, 0b10_0110),
    (571_250_000, 673_750_000, 0b10_1110),
    (673_750_000, 810_000_000, 0b11_0110),
    (810_000_000, 930_000_000, 0b11_1110),
    (930_000_000, 1_142_500_000, 0b10_0101),
    (1_142_500_000, 1_347_500_000, 0b10_1101),
    (1_347_500_000, 1_620_000_000, 0b11_0101),
    (1_620_000_000, 1_860_000_000, 0b11_1101),
    (1_860_000_000, 2_285_000_000, 0b10_0100),
    (2_285_000_000, 2_695_000_000, 0b10_1100),
    (2_695_000_000, 3_240_000_000, 0b11_0100),
    (3_240_000_000, 3_720_000_000, 0b11_1100),
];

fn freqsel(freq: u32) -> Result<u8> {
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

fn freq_to_params(refclk: u32, freq: u32) -> Result<TuningParams> {
    // For wanted LO frequency f_lo and given PLL reference clock
    // frequency f_ref, calculate integer and fractional part of the
    // divider as below.

    // First, find temporary variable x from the 3 least significant
    // bits of the FREQSEL value:
    let freqsel = freqsel(freq)?;
    let x = 2u32.pow((u32::from(freqsel) & 0b111) - 3);
    // Use x to calculate NINT and NFRAC:
    let nint = (x * freq) / refclk;
    let nfrac =
        2f64.powi(23) * ((f64::from(x) * f64::from(freq)) / f64::from(refclk) - f64::from(nint));
    Ok(TuningParams {
        freqsel,
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
    let freq = 2_140_000_000;
    let TuningParams {
        freqsel,
        nint,
        nfrac,
    } = freq_to_params(REFCLK, freq).unwrap();
    assert_eq!(freqsel, 0b100100);
    assert_eq!(nint, 139);
    assert_eq!(nfrac, 2708821);
}
