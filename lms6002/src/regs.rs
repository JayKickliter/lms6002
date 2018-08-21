//! # Registers
//!
//! This module contains types representing LMS6002 registers. See the
//! [LMS6002 Programming and Calibration Guide](https://github.com/myriadrf/LMS6002D-docs/blob/master/LMS6002Dr2-Programming%20and%20Calibration%20Guide-1_1r4.pdf)
//! for further detail.
//!
//! ## Memory map
//!
//! | Address (7 bits) | Description                           |
//! |------------------|---------------------------------------|
//! | `x000:xxxx`      | Top level configuration               |
//! | `x001:xxxx`      | TX PLL configuration                  |
//! | `x010:xxxx`      | RX PLL configuration                  |
//! | `x011:xxxx`      | TX LPF modules configuration          |
//! | `x100:xxxx`      | TX RF modules configuration           |
//! | `x101:xxxx`      | RX LPF, DAC/ADC modules configuration |
//! | `x110:xxxx`      | RX VGA2 configuration                 |
//! | `x111:xxxx`      | RX FE modules configuration           |

use std::fmt::Debug;
use std::convert::{From, Into};

pub trait LmsReg: Debug + From<u8> + Into<u8> {
    const OFFSET: u8;
    fn addr() -> u8 {
        Self::OFFSET
    }
}

bitfield!{
    pub struct Top00(u8);
    impl Debug;

    /// Value from DC calibration module selected by `DC_ADDR`.
    pub dc_regval, _: 5, 0;
}

bitfield!{
    pub struct Top01(u8);
    impl Debug;

    /// Value of the cal_core block in the LPF which calibrates the RC
    /// time constant.
    ///
    /// It should be read by software to set the value of the TIA
    /// feedback cap (`CFB_RXFE_TIA`).
    pub rccal_lpfcal, _: 7, 5;

    /// Lock pattern register.
    ///
    /// Locked, when register value is not "000" nor "111".
    pub dc_lock, _: 4, 2;

    /// Indicates calibration status.
    /// - 1: calibration in progress.
    /// - 0: calibration is done.
    pub dc_clbr_done, _: 1;

    /// Value from DC module comparator, selected by `DC_ADDR`.
    ///
    /// - 1: Count Up;
    /// - 0: Count Down.
    pub dc_ud, _: 0;
}

bitfield!{
    pub struct Top02(u8);
    impl Debug;

    /// Value to load into selected (by `DC_ADDR`) DC calibration module.
    pub dc_cntval, set_dc_cntval: 5, 0;
}

bitfield!{
    pub struct Top03(u8);
    impl Debug;

    /// Start calibration command of the module, selected by `DC_ADDR`.
    /// - 1: Start Calibration;
    /// - 0: Deactivate Start Calibration command. (default)
    pub dc_start_clbr, set_dc_start_clbr: 5;


    /// Load value from `DC_CNTVAL` to module, selected by `DC_ADDR`.
    /// - 1: Load Value;
    /// - 0: Deactivate Load Value command. (default)
    pub dc_load, set_dc_load: 4;

    /// resets all DC Calibration modules.
    /// - 1: Reset inactive (default)
    /// - 0: Reset active.
    pub dc_sreset, set_dc_sreset: 3;

    /// Active calibration module address.
    /// - 000: LPF tuning module.
    /// - 001...111: Not used
    pub dc_addr, set_dc_addr: 2, 0;
}

bitfield!{
    pub struct Top04(u8);
    impl Debug;

    /// Chip version.
    pub ver, _: 7, 4;

    /// Chip revision.
    pub rev, _: 3, 0;
}

bitfield!{
    pub struct Top05(u8);
    impl Debug;

    /// - 0: decode control signals (default)
    /// - 1: use control signals from test mode registers
    pub decode, set_decode: 7;

    /// DSM soft reset.
    /// - 0: reset state
    /// - 1: inactive (default)
    pub sreset, set_sreset: 5;

    /// Top modules enable.
    /// - 0: Top modules powered down
    /// - 1: Top modules enabled (default)
    pub en, set_en: 4;

    /// Soft transmit enable.
    /// - 0: Transmitter powered down (default)
    /// - 1: Transmitter enabled
    pub stxen, set_stxen: 3;

    // Soft receive enable.
    // - 0: Receiver powered down (default)
    // - 1: Receiver enabled
    pub srxen, set_srxen: 2;

    // Serial port mode.
    // - 0: three wire mode
    // - 1: four wire mode (default)
    pub tfwmode, set_tfwmode: 1;
}

bitfield!{
    pub struct Top06(u8);
    impl Debug;

    /// Select the clock for LPF tuning module.
    /// - 0: 40 MHz clock generated from TX PLL output
    /// - 1: use PLL reference clock (default)
    pub clksel_lpfcal, set_clksel_lpfcal: 3;

    /// Power down on chip LPF tuning clock generation block.
    /// - 0: powered up
    /// - 1: powered down (default)
    pub pd_clklpfcal, set_pd_clklpfcal: 2;

    /// Enables the enforce mode.Passes `FORCE_CODE_CAL_LPFCAL` to `RCCAL_LPFCAL`.
    /// - 0: enforce mode disabled (default)
    /// - 1: enforce mode enabled
    pub enf_en_cal_lpfcal, set_enf_en_cal_lpfcal: 1;

    /// Reset signal used at the beginning of calibration cycle.
    ///
    /// NOTE: Reset signal needs to be longer than 100ns.
    /// - 0: normal state
    /// - 1: reset state (default)
    pub rst_cal_lpfcal, set_rst_cal_lpfcal: 0;
}

// bitfield!{
//     pub struct Top0(u8);
//     impl Debug;
// }
// bitfield!{
//     pub struct Top0(u8);
//     impl Debug;
// }
// bitfield!{
//     pub struct Top0(u8);
//     impl Debug;
// }
// bitfield!{
//     pub struct Top0(u8);
//     impl Debug;
// }
// bitfield!{
//     pub struct Top0(u8);
//     impl Debug;
// }
// bitfield!{
//     pub struct Top0(u8);
//     impl Debug;
// }
// bitfield!{
//     pub struct Top0(u8);
//     impl Debug;
// }
// bitfield!{
//     pub struct Top0(u8);
//     impl Debug;
// }


bitfield!{
    pub struct Pll00(u8);
    impl Debug;

    /// Integer part of the divider (MSBs)
    pub nint_8_1, set_nint_8_1: 7, 0;
}

bitfield!{
    pub struct Pll01(u8);
    impl Debug;

    /// Integer part of the divider (LSB)
    pub nint_0, set_nint_0: 7;

    /// Fractional part of the divider
    pub nfrac_22_16, set_nfrac_22_16: 6, 0;
}

bitfield!{
    pub struct Pll02(u8);
    impl Debug;

    /// Fractional part of the divider
    pub nfrac_15_8, set_nfrac_15_8: 7, 0;
}

bitfield!{
    pub struct Pll03(u8);
    impl Debug;

    /// Fractional part of the divider
    pub nfrac_7_0, set_nfrac_7_0: 7, 0;
}

bitfield!{
    pub struct Pll04(u8);
    impl Debug;

    /// Dithering control
    /// - 0: disabled
    /// - 1: enabled (default)
    pub dithen, set_dithen: 7;

    /// How many bits to dither if DITHEN=1
    /// - 000: 1 bit (default)
    /// - 001: 2 bits
    /// - 010: 3 bits
    /// - ...
    /// - 111: 8 bits
    pub dithn, set_dithn: 6, 4;

    /// PLL enable
    /// - 0: PLL powered down
    /// - 1: PLL enabled (default)

    pub en, set_en: 3;

    /// Delta sigma auto bypass when NFRAC = 0
    /// - 0: disabled (default)
    /// - 1: enabled
    pub autobyp, set_autobyp: 2;

    /// - 0: decode power down/enable signals (default)
    /// - 1: use power down/enable signals from test mode registers
    pub decode, set_decode: 1;
}

bitfield!{
    pub struct Pll05(u8);
    impl Debug;

    /// VCO selection
    /// - 000: All VCOs powered down
    /// - 100: Low frequency VCO (vco4)
    /// - 101: Mid low frequency VCO (vco3) (default)
    /// - 110: Mid high frequency VCO (vco2)
    /// - 111: High frequency VCO (vco1)
    pub selvco, set_selvco: 7, 4;

    /// PLL output frequency range selection
    /// - 000: All dividers powered down
    /// - 100: Fvco/2 (2-4GHz range) (default)
    /// - 101: Fvco/4 (1-2GHz range)
    /// - 110: Fvco/8 (0.5-1GHz range)
    /// - 111: Fvco/16 (0.25-0.5GHz range)
    pub frange, set_frange: 4, 2;

    /// Select output buffer in RX PLL, not used in TX PLL
    /// - 00: All output buffers powered down
    /// - 01: First buffer enabled for LNA1 path (default) 10: Second buffer enabled for LNA2 path
    /// - 11: Third buffer enabled for LNA3 path
    pub selout, set_selout: 1, 0;
}

bitfield!{
    pub struct Pll06(u8);
    impl Debug;

    /// Enable PFD UP pulses
    /// - 0: disabled
    /// - 1: enabled (default)
    pub en_pfd_up, set_en_pfd_up: 7;

    /// - 0: Test signal output buffer disabled (default)
    /// - 1: Test signal output buffer enabled
    pub oen_tstd_sx, set_oen_tstd_sx: 6;

    /// - 0: Test signal pass disabled (default)
    /// - 1: Test signal pass enabled
    pub passen_tstod_sd, set_passen_tstod_sd: 5;

    /// Charge pump current. Binary coded, LSB = 100uA:
    /// - 00000: 0uA
    /// - 00001: 100uA
    /// - ...
    /// - 11000: 2400uA
    /// - ...
    /// - 2400 uA
    pub ichp, set_ichp: 4, 0;
}

bitfield!{
    pub struct Pll07(u8);
    impl Debug;

    /// Bypass VCO regulator
    /// - 0: not bypassed
    /// - 1: regulator bypassed (default)
    pub bypvcoreg, set_bypvcoreg: 7;

    ///
    /// VCO regulator power down.
    /// - 0: regulator powered up
    /// - 1: regulator powered down (default)
    pub pdvcoreg, set_pdvcoreg: 6;

    /// VCO regulator band gap settling time control.
    ///
    /// Shorts the resistor in band gap to speed up charging for faster response. After the initial charge up, it should be disabled.
    /// - 1: resistor shorted (default)
    /// - 0: switch open
    pub fstvcobg, set_fstvcobg: 5;

    /// Charge pump UP offset current. Binary coded, LSB = 10uA:
    /// - 00000: 0uA
    /// - 00001: 10uA
    /// - ...
    /// - 11000: 240uA
    /// - ...: 240uA
    pub offup, set_offup: 5, 0;
}

bitfield!{
    pub struct Pll08(u8);
    impl Debug;

    /// VCO regulator output voltage control, 3 MSBs.
    ///
    /// LSB=100mV, VOVCOREG{3:0} coded as below
    /// - 0000: 1.4V, min output
    /// - ...
    /// - 0101: 1.9V (default)
    /// - ...
    /// - 1100: 2.6V, max output
    /// - 1101, 1110, 1111, not valid codes
    pub vovcoreg, set_vovcoreg: 7, 5;

    /// Charge pump DOWN offset current.
    ///
    /// Binary coded, LSB = 10uA:
    /// - 00000: 0uA
    /// - 00001: 10uA
    /// - ...
    /// - 11000: 240uA
    /// - ...: 240uA
    pub offdown, set_offdown: 4, 0;
}

bitfield!{
    pub struct Pll09(u8);
    impl Debug;

    /// VCO regulator output voltage control, LSB
    pub vovcoreg, set_vovcoreg: 7;

    /// Switch capacitance programming. Binary coded.
    /// - 000000 (max capacitance, min frequency)
    /// - 010100 (default)
    /// - 111111 (min capacitance, max frequency)
    pub vcocap, set_vcocap: 5, 0;
}

bitfield!{
    pub struct Pll10(u8);
    impl Debug;

    /// Value from Vtune comparator (Read Only)
    pub vtune_h, _: 7;

    /// Value from Vtune comparator (Read Only)
    pub vtune_l, _: 6;
}

bitfield!{
    pub struct Pll11(u8);
    impl Debug;

    /// VCO Comparator Enable
    /// - 0: enabled (powered up) (default)
    /// - 1: disabled (powered down)
    pub pd_vcocomp_sx, set_pd_vcocomp_sx: 3;
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_debug() {
        let mut reg = Pll04(0);
        reg.set_dithen(true);
        println!("{:#?}", reg);
    }
}
