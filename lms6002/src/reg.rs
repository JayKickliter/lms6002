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

use std::convert::{From, Into};
use std::fmt::Debug;
use std::marker::PhantomData;

pub use self::adcdac::*;
pub use self::pll::*;
pub use self::rxfe::*;
pub use self::rxvga2::*;
pub use self::top::*;
pub use self::txlpf::*;
pub use self::txrf::*;

pub trait LmsReg: Debug + From<u8> + Into<u8> + Copy {
    fn addr() -> u8;
}

macro_rules! lmsreg {
    // lmsreg!(Top00, 0);
    ($reg:tt, $offset:expr) => {
        impl ::std::convert::From<u8> for $reg {
            fn from(val: u8) -> $reg {
                $reg(val)
            }
        }

        impl ::std::convert::From<$reg> for u8 {
            fn from(val: $reg) -> u8 {
                val.0
            }
        }

        impl LmsReg for $reg {
            fn addr() -> u8 {
                $offset
            }
        }
    };
}

pub trait DcCalStatusReg {
    fn lock(&self) -> bool;
    fn clbr_done(&self) -> bool;
}

macro_rules! dc_cal_status_reg {
    ($reg:tt) => {
        impl DcCalStatusReg for $reg {
            fn lock(&self) -> bool {
                let val = self.dc_lock();
                (val != 0b000) && (val != 0b111)
            }

            fn clbr_done(&self) -> bool {
                // For some stupid reason, this field defines DONE as
                // 0, and 1 as not DONE. Let's negate it and return a
                // proper `bool`.
                !self.dc_clbr_done()
            }
        }
    };
}

pub trait DcCalControlReg {
    fn set_start_clbr(&mut self, start: bool);
    fn set_addr(&mut self, addr: u8);
}

macro_rules! dc_cal_control_reg {
    ($reg:tt) => {
        impl DcCalControlReg for $reg {
            fn set_start_clbr(&mut self, start: bool) {
                self.set_dc_start_clbr(start);
            }

            fn set_addr(&mut self, addr: u8) {
                self.set_dc_addr(addr);
            }
        }
    };
}

mod top {
    ////////////////////////////////////////////////////////////////////////
    // Top Level Registers                                                //
    ////////////////////////////////////////////////////////////////////////

    use super::*;

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Top0x00(u8);
        impl Debug;

        /// Value from DC calibration module selected by `DC_ADDR`.
        pub dc_regval, _: 5, 0;
    }
    lmsreg!(Top0x00, 0x00);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Top0x01(u8);
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
    lmsreg!(Top0x01, 0x01);
    dc_cal_status_reg!(Top0x01);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Top0x02(u8);
        impl Debug;

        /// Value to load into selected (by `DC_ADDR`) DC calibration module.
        pub dc_cntval, set_dc_cntval: 5, 0;
    }
    lmsreg!(Top0x02, 0x02);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Top0x03(u8);
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
    lmsreg!(Top0x03, 0x03);
    dc_cal_control_reg!(Top0x03);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Top0x04(u8);
        impl Debug;

        /// Chip version.
        pub ver, _: 7, 4;

        /// Chip revision.
        pub rev, _: 3, 0;
    }
    lmsreg!(Top0x04, 0x04);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Top0x05(u8);
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
    lmsreg!(Top0x05, 0x05);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Top0x06(u8);
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
    lmsreg!(Top0x06, 0x06);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Top0x07(u8);
        impl Debug;

        /// Enable signal.
        ///
        /// Should be enabled only during the RC calibration algorithm running.
        /// - 0: Block disabled (default)
        /// - 1: Block enabled
        pub en_cal_lpfcal, set_en_cal_lpfcal: 7;

        /// Input code coming from software.
        ///
        /// Will be passed to the output if `ENF_EN_CAL_LPFCAL=1`.
        /// - 000 (default)
        pub force_code_cal_lpfcal, set_force_code_cal_lpfcal: 6, 4;

        /// LPF bandwidth control (Set this code to RXLPF BWC if RXLPF and TXLPF have different cut-off frequencies).
        ///
        /// | code | Bandwidth [MHz] |
        /// |------|-----------------|
        /// | 0000 |  14 (default)   |
        /// | 0001 |  10             |
        /// | 0010 |  7              |
        /// | 0011 |  6              |
        /// | 0100 |  5              |
        /// | 0101 |  4.375          |
        /// | 0110 |  3.5            |
        /// | 0111 |  3              |
        /// | 1000 |  2.75           |
        /// | 1001 |  2.5            |
        /// | 1010 |  1.92           |
        /// | 1011 |  1.5            |
        /// | 1100 |  1.375          |
        /// | 1101 |  1.25           |
        /// | 1110 |  0.875          |
        /// | 1111 |  0.75           |
        pub bwc_lpfcal, set_bwc_lpfcal: 3, 0;
    }
    lmsreg!(Top0x07, 0x07);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Top0x08(u8);
        impl Debug;

        /// BB loopback enable.
        /// - 1: TX BB loopback signal is connected to RXLPF input. If enabled, RXTIA should be disabled (powered down)
        /// - 0: default value
        pub lben_lpfin, set_lben_lpfin: 6;

        /// BB loopback enable.
        /// - 1: TX BB loopback signal is connected to RXVGA2 input. If enabled, LPF should be disabled (powered down).
        /// - 0: default value.
        pub lben_vga2in, set_lben_vga2in: 5;

        /// BB loopback enable.
        /// - 1: TX BB loopback signal is connected to the RX output pins. If enabled, RXLPF and RXVGA2 should be disabled (powered down)
        /// - 0: default value.
        pub lben_opin, set_lben_opin: 4;

        /// RF loop back control.
        ///
        /// When activated, LNAs should be disabled (powered down).
        /// - 0:    RF loopback disabled (default)
        /// - 1:    TXMIX output connected to LNA1 path
        /// - 2:    TXMIX output connected to LNA2 path
        /// - 3:    TXMIX output connected to LNA3 path
        /// - 4-15: Reserved. Not valid for settings.
        pub lbrfen, set_lbrfen: 3, 0;
    }
    lmsreg!(Top0x08, 0x08);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Top0x09(u8);
        impl Debug;

        // RX out/ADC in high-Z switch control
        // - 0: switch open (RX output/ADC input chip pins disconnected) (default)
        // - 1: switch closed, RXVGA2 should be powered off first
        pub rxoutsw, set_rxoutsw: 7;

        /// - 1 PLLCLKOUT enabled (default)
        /// - 0 PLLCLKOUT disabled
        pub pllclkout_en, set_pllclkout_en: 6;

        /// - 1: LPF CAL clock enabled
        /// - 0: LPF CAL clock disabled (default)
        pub lpf_cal_clk_en, set_lpf_cal_clk_en: 5;

        /// - 1: Rx VGA2 DCCAL clock enabled
        /// - 0: Rx VGA2 DCCAL clock disabled (default)
        pub rx_vga2_dccal_en, set_rx_vga2_dccal_en: 4;

        /// - 1: Rx LPF DCCAL clock enabled
        /// - 0: Rx LPF DCCAL clock disabled (default)
        pub rx_lpf_dccal_clk_en, set_rx_lpf_dccal_clk_en: 3;

        /// - 1: Rx DSM SPI clock enabled
        /// - 0: Rx DSM SPI clock disabled (default)
        pub rx_dsm_spi_clk_en, set_rx_dsm_spi_clk_en: 2;

        /// - 1: Tx LPF SPI DCCAL clock enabled
        /// - 0: Tx LPF SPI DCCAL clock disabled (default)
        pub tx_lpf_spi_dccal_clk_en, set_tx_lpf_spi_dccal_clk_en: 1;

        /// - 1: Tx DSM SPI clock enabled
        /// - 0: Tx DSM SPI clock disabled (default)
        pub tx_dsm_spi_clk_en, set_tx_dsm_spi_clk_en: 0;
    }
    lmsreg!(Top0x09, 0x09);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Top0x0A(u8);
        impl Debug;

        /// Frequency/Time division duplexing selection
        /// - 0: FDD mode (default)
        /// - 1: TDD mode
        pub fddtdd, set_fddtdd: 1;

        /// TDD mode selection if FDDTDD=1
        /// - 0: TDD Transmit mode (default)
        /// - 1: TDD Receive mode
        pub tddmod, set_tddmod: 0;
    }
    lmsreg!(Top0x0A, 0x0A);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Top0x0B(u8);
        impl Debug;

        /// XCO buffer power down
        /// - 0: buffer powered up (default)
        /// - 1: buffer powered down
        pub pdxcobuf, set_pdxcobuf: 4;

        /// XCO buffer self-biasing control
        /// - 0: self biasing disabled
        /// - 1: self biasing enabled (default)
        pub slfbxcobuf, set_slfbxcobuf: 3;

        /// XCO buffer bypass
        /// - 0: buffer active (default)
        /// - 1: buffer bypassed
        pub bypxcobuf, set_bypxcobuf: 2;

        /// - 1: PD_DCOREF_LPFCAL powered down
        /// - 0: PD_DCOREF_LPFCAL powered up (default)
        pub pd_dcoref_lpfcal, set_pd_dcoref_lpfcal: 1;

        /// - 1: RF loop back switch powered up
        /// - 0: RF loop back switch powered down (default)
        pub pu_rf_lbs, set_pu_rf_lbs: 0;
    }
    lmsreg!(Top0x0B, 0x0B);
}

mod pll {
    ////////////////////////////////////////////////////////////////////////
    // PLL Registers                                                      //
    ////////////////////////////////////////////////////////////////////////
    use super::*;

    /// A Represents either the TX or RX PLL modules.
    pub trait PllMod: Copy {
        /// Absolute offset of module in LMS6002 address space.
        const OFFSET: u8;
    }

    #[derive(Clone, Copy)]
    pub struct TxPll;
    impl PllMod for TxPll {
        const OFFSET: u8 = 0x10;
    }

    #[derive(Clone, Copy)]
    pub struct RxPll;
    impl PllMod for RxPll {
        const OFFSET: u8 = 0x20;
    }

    #[derive(Clone, Copy)]
    pub struct PllReg<R: Debug + Copy, M: PllMod>(pub R, PhantomData<M>);

    impl<R: Debug + Copy, M: PllMod> PllReg<R, M> {
        pub fn new(r: R, _: M) -> Self {
            PllReg(r, PhantomData)
        }
    }

    macro_rules! pllreg {
        ($reg:tt, $offset:expr) => {
            impl<M: PllMod> ::std::convert::AsRef<$reg> for PllReg<$reg, M> {
                fn as_ref(&self) -> &$reg {
                    &self.0
                }
            }

            impl<M: PllMod> ::std::convert::From<u8> for PllReg<$reg, M> {
                fn from(val: u8) -> Self {
                    PllReg($reg(val), PhantomData)
                }
            }

            impl<M: PllMod> ::std::convert::From<PllReg<$reg, M>> for u8 {
                fn from(val: PllReg<$reg, M>) -> Self {
                    (val.0).0
                }
            }

            impl<M: PllMod> Debug for PllReg<$reg, M> {
                fn fmt(&self, f: &mut ::std::fmt::Formatter) -> Result<(), ::std::fmt::Error> {
                    self.0.fmt(f)
                }
            }

            impl<M: PllMod> LmsReg for PllReg<$reg, M> {
                fn addr() -> u8 {
                    M::OFFSET + $offset
                }
            }
        };
    }

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Pll0x00(u8);
        impl Debug;

        /// Integer part of the divider (MSBs)
        pub nint_8_1, set_nint_8_1: 7, 0;
    }
    pllreg!(Pll0x00, 0x00);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Pll0x01(u8);
        impl Debug;

        /// Integer part of the divider (LSB)
        pub nint_0, set_nint_0: 7, 7;

        /// Fractional part of the divider
        pub nfrac_22_16, set_nfrac_22_16: 6, 0;
    }
    pllreg!(Pll0x01, 0x01);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Pll0x02(u8);
        impl Debug;

        /// Fractional part of the divider
        pub nfrac_15_8, set_nfrac_15_8: 7, 0;
    }
    pllreg!(Pll0x02, 0x02);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Pll0x03(u8);
        impl Debug;

        /// Fractional part of the divider
        pub nfrac_7_0, set_nfrac_7_0: 7, 0;
    }
    pllreg!(Pll0x03, 0x03);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Pll0x04(u8);
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
    pllreg!(Pll0x04, 0x04);

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum SelVco {
        Off,
        Vco4,
        Vco3,
        Vco2,
        Vco1,
        Inv(u8),
    }

    impl From<SelVco> for u8 {
        fn from(val: SelVco) -> u8 {
            match val {
                SelVco::Off => 0b000,
                SelVco::Vco4 => 0b100,
                SelVco::Vco3 => 0b101,
                SelVco::Vco2 => 0b110,
                SelVco::Vco1 => 0b111,
                SelVco::Inv(inv) => inv,
            }
        }
    }

    impl From<u8> for SelVco {
        fn from(val: u8) -> SelVco {
            match val {
                0b000 => SelVco::Off,
                0b100 => SelVco::Vco4,
                0b101 => SelVco::Vco3,
                0b110 => SelVco::Vco2,
                0b111 => SelVco::Vco1,
                inv => SelVco::Inv(inv),
            }
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum FRange {
        Off,      // 0b000,
        VcoDiv2,  // 0b100,
        VcoDiv4,  // 0b101,
        VcoDiv8,  // 0b110,
        VcoDiv16, // 0b111,
        Inv(u8),
    }

    impl From<FRange> for u8 {
        fn from(val: FRange) -> u8 {
            match val {
                FRange::Off => 0b000,
                FRange::VcoDiv2 => 0b100,
                FRange::VcoDiv4 => 0b101,
                FRange::VcoDiv8 => 0b110,
                FRange::VcoDiv16 => 0b111,
                FRange::Inv(inv) => inv,
            }
        }
    }

    impl From<u8> for FRange {
        fn from(val: u8) -> FRange {
            match val {
                0b000 => FRange::Off,
                0b100 => FRange::VcoDiv2,
                0b101 => FRange::VcoDiv4,
                0b110 => FRange::VcoDiv8,
                0b111 => FRange::VcoDiv16,
                inv => FRange::Inv(inv),
            }
        }
    }

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Pll0x05(u8);
        impl Debug;

        /// VCO selection
        /// - 000: All VCOs powered down
        /// - 100: Low frequency VCO (vco4)
        /// - 101: Mid low frequency VCO (vco3) (default)
        /// - 110: Mid high frequency VCO (vco2)
        /// - 111: High frequency VCO (vco1)
        pub u8, into SelVco, selvco, set_selvco: 7, 5;

        /// PLL output frequency range selection
        /// - 000: All dividers powered down
        /// - 100: Fvco/2 (2-4GHz range) (default)
        /// - 101: Fvco/4 (1-2GHz range)
        /// - 110: Fvco/8 (0.5-1GHz range)
        /// - 111: Fvco/16 (0.25-0.5GHz range)
        pub u8, into FRange, frange, set_frange: 4, 2;

        /// Select output buffer in RX PLL, not used in TX PLL
        /// - 00: All output buffers powered down
        /// - 01: First buffer enabled for LNA1 path (default) 10: Second buffer enabled for LNA2 path
        /// - 11: Third buffer enabled for LNA3 path
        pub selout, set_selout: 1, 0;
    }
    pllreg!(Pll0x05, 0x05);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Pll0x06(u8);
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
    pllreg!(Pll0x06, 0x06);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Pll0x07(u8);
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
    pllreg!(Pll0x07, 0x07);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Pll0x08(u8);
        impl Debug;

        /// VCO regulator output voltage control, 3 MSBs.
        ///
        /// LSB=100mV, VOVCOREG[3:0] coded as below
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
    pllreg!(Pll0x08, 0x08);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Pll0x09(u8);
        impl Debug;

        /// VCO regulator output voltage control, LSB
        pub vovcoreg, set_vovcoreg: 7;

        /// Switch capacitance programming. Binary coded.
        /// - 000000 (max capacitance, min frequency)
        /// - 010100 (default)
        /// - 111111 (min capacitance, max frequency)
        pub vcocap, set_vcocap: 5, 0;
    }
    pllreg!(Pll0x09, 0x09);

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum VTune {
        Low,     // 0b10
        InRange, // 0b00
        High,    // 0b01
        Inv(u8),
    }

    impl From<u8> for VTune {
        fn from(val: u8) -> VTune {
            match val {
                0b01 => VTune::Low,
                0b00 => VTune::InRange,
                0b10 => VTune::High,
                inv => VTune::Inv(inv),
            }
        }
    }

    impl From<VTune> for u8 {
        fn from(val: VTune) -> u8 {
            match val {
                VTune::Low => 0b01,
                VTune::InRange => 0b00,
                VTune::High => 0b10,
                VTune::Inv(inv) => inv,
            }
        }
    }

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Pll0x0A(u8);
        impl Debug;

        /// Value from Vtune comparator (Read Only)
        pub vtune_h, _: 7;

        /// Value from Vtune comparator (Read Only)
        pub vtune_l, _: 6;

        /// Combination of [`vtune_h`] [`vtune_l`] in a single type
        pub u8, into VTune, vtune, _: 7, 6;
    }
    pllreg!(Pll0x0A, 0x0A);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct Pll0x0B(u8);
        impl Debug;

        /// VCO Comparator Enable
        /// - 0: enabled (powered up) (default)
        /// - 1: disabled (powered down)
        pub pd_vcocomp_sx, set_pd_vcocomp_sx: 3;
    }
    pllreg!(Pll0x0B, 0x0B);
}

mod txlpf {
    ////////////////////////////////////////////////////////////////////////
    // TX LPF Registers                                                   //
    ////////////////////////////////////////////////////////////////////////
    pub use super::*;

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxLpf0x30(u8);
        impl Debug;

        /// Value from DC calibration module selected by DC_ADDR.
        pub dc_regval, _: 5, 0;
    }
    lmsreg!(TxLpf0x30, 0x30);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxLpf0x31(u8);
        impl Debug;

        /// Lock pattern register.
        /// Locked, when register value is neither "000" nor "111".
        pub dc_lock, _: 4, 2;

        /// indicates calibration status.
        /// - 1: Calibration in progress.
        /// - 0: Calibration is done.
        pub dc_clbr_done, _: 1;

        /// Value from DC module comparator, selected by DC_ADDR.
        /// - 1: Count Up.
        /// - 0: Count Down.
        pub dc_ud, _: 0;
    }
    lmsreg!(TxLpf0x31, 0x31);
    dc_cal_status_reg!(TxLpf0x31);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxLpf0x32(u8);
        impl Debug;

        /// Value to load into selected (by DC_ADDR) DC calibration module.
        pub dc_cntval, set_dc_cntval: 5, 0;
    }
    lmsreg!(TxLpf0x32, 0x32);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxLpf0x33(u8);
        impl Debug;

        /// Start calibration command of module selected by DC_ADDR.
        /// - 1: Start calibration.
        /// - 0: Deactivate start calibration command. **(Default)**
        pub dc_start_clbr, set_dc_start_clbr: 5;

        /// Load value from DC_CNTVAL to module, selected by DC_ADDR.
        /// - 1: Load Value.
        /// - 0: Deactivate Load Value command. **(Default)**
        pub dc_load, set_dc_load: 4;

        /// Resets all DC Calibration modules.
        /// - 1: Reset inactive. **(Default)**
        /// - 0: Reset active.
        pub dc_sreset, set_dc_sreset: 3;

        /// Active calibration module address.
        /// - 000: I filter.
        /// - 001: Q filter.
        /// - 010: 111 Not used.
        pub dc_addr, set_dc_addr: 2, 0;
    }
    lmsreg!(TxLpf0x33, 0x33);
    dc_cal_control_reg!(TxLpf0x33);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxLpf0x34(u8);
        impl Debug;

        /// LPF bandwidth control.
        ///
        /// code | Bandwidth [MHz]
        /// -----|----------------
        /// 0000 | 14 **(Default)**
        /// 0001 | 10
        /// 0010 | 7
        /// 0011 | 6
        /// 0100 | 5
        /// 0101 | 4.375
        /// 0110 | 3.5
        /// 0111 | 3
        /// 1000 | 2.75
        /// 1001 | 2.5
        /// 1010 | 1.92
        /// 1011 | 1.5
        /// 1100 | 1.375
        /// 1101 | 1.25
        /// 1110 | 0.875
        /// 1111 | 0.75
        pub bwc_lpf, set_bwc_lpf: 5, 2;

        /// LPF modules enable.
        /// - 0: LPF modules powered down.
        /// - 1: LPF modules enabled. **(Default)**
        pub en, set_en: 1;

        /// Decode.
        /// - 0: Decode control signals. **(Default)**
        /// - 1: Use control signals from test mode registers.
        pub decode, set_decode: 0;
    }
    lmsreg!(TxLpf0x34, 0x34);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxLpf0x35(u8);
        impl Debug;

        /// LPF bypass enable.
        /// - 1: Bypass switches will bypass the LPF.
        /// - 0: Normal operation. **(Default)**
        pub byp_en_lpf, set_byp_en_lpf: 6;

        /// Resistor calibration control for the DC offset cancellation DAC.
        /// - 001100: **(Default)**
        pub dco_daccal, set_dco_daccal: 5, 0;
    }
    lmsreg!(TxLpf0x35, 0x35);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxLpf0x36(u8);
        impl Debug;

        /// TX data DAC buffers power down.
        /// - 0: Enabled. **(Default)**
        /// - 1: Powered Down.
        pub tx_dacbuf_pd, set_tx_dacbuf_pd: 7;

        /// Calibration value, coming from TRX_LPF_CAL module.
        /// - 011: **(Default)**
        pub rccal_lpf, set_rccal_lpf: 6, 4;

        /// Power down for the DAC in the DC offset cancellation block.
        /// - 1: Powered Down.
        /// - 0: Enabled. **(Default)**
        pub pd_dcodac_lpf, set_pd_dcodac_lpf: 2;

        /// Power down signal for the dc_ref_con3 block.
        /// - 1: Powered Down.
        /// - 0: Enabled. **(Default)**
        pub pd_dcoref_lpf, set_pd_dcoref_lpf: 1;

        /// Power down for the filter.
        /// - 1: Powered Down.
        /// - 0: Enabled. **(Default)**
        pub pd_fil_lpf, set_pd_fil_lpf: 0;
    }
    lmsreg!(TxLpf0x36, 0x36);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxLpf0x3F(u8);
        impl Debug;

        /// Power down DC offset comparators in DC offset cancellation block. Should be powered up only when DC offset cancellation algorithm is running.
        /// - 1: Powered Down.
        /// - 0: Enabled. **(Default)**
        pub pd_dcocmp_lpf, set_pd_dcocmp_lpf: 7;
    }
    lmsreg!(TxLpf0x3F, 0x3F);
}

mod txrf {
    ////////////////////////////////////////////////////////////////////////
    // TX RF modules configuration                                        //
    ////////////////////////////////////////////////////////////////////////
    pub use super::*;

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxRf0x40(u8);
        impl Debug;

        /// TXRF modules enable
        /// - 0: TXRF modules powered down.
        /// - 1: TXRF modules enabled. **(Default)**
        pub en, set_en: 1;

        /// Decode:
        /// - 0: Decode control signals. **(Default)**
        /// - 1: Use control signals from test mode registers.
        pub decode, set_decode: 0;
    }
    lmsreg!(TxRf0x40, 0x40);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxRf0x41(u8);
        impl Debug;

        /// TXVGA1 gain, log-linear control. LSB=1dB, encoded as shown below.
        /// Code  | Gain [dB]
        /// ------|--------------
        /// 00000 | -35
        /// 00001 | -34
        /// …     | …
        /// 10101 | -14 **(Default)**
        /// …	  | …
        /// 11110 | -5
        /// 11111 | -4
        pub vga1gain, set_vga1gain: 4, 0;
    }
    lmsreg!(TxRf0x41, 0x41);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxRf0x42(u8);
        impl Debug;

        /// TXVGA1 DC shift control, LO leakage cancellation. LSB=0.0625mV, encoded as shown below.
        /// Code     | DC Shift [mV]
        /// ---------|---------------
        /// 00000000 | -16
        /// …        | …
        /// 01111111 | -0.0625
        /// 10000000 | 0 **(Default)**
        /// 10000001 | 0.0625
        /// …        | …
        /// 11111111 | 15.9375
        pub vga1dc_i, set_vga1dc_i: 7, 0;
    }
    lmsreg!(TxRf0x42, 0x42);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxRf0x43(u8);
        impl Debug;

        /// TXVGA1 DC shift control, LO leakage cancellation LSB=0.0625mV, encoded as shown below.
        /// Code     | DC Shift [mV]
        /// ---------|--------------
        /// 00000000 | -16
        /// …        | …
        /// 01111111 | -0.0625
        /// 10000000 | 0 **(Default)**
        /// 10000001 | 0.0625
        /// …        | …
        /// 11111111 | 15.9375
        pub vga1dc_q, set_vga1dc_q: 7, 0;
    }
    lmsreg!(TxRf0x43, 0x43);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxRf0x44(u8);
        impl Debug;

        /// VGA2 power amplifier (TX output) selection.
        /// PA_EN{2:1] | PA1 | PA2
        /// -----------|-----|--------------
        /// 00         | OFF | OFF
        /// 01         | ON  | OFF **(Default)**
        /// 10         | OFF | ON
        /// 11         | OFF | OFF
        pub pa_vga2_en, set_vga2_en: 4, 2;

        /// AUXPA, auxiliary (RF loopack) PA power down.
        /// - 0: Powered up. **(Default)**
        /// - 1: Powered down.
        pub pa_augpa_en, set_augpa_en: 2;
    }
    lmsreg!(TxRf0x44, 0x44);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxRf0x45(u8);
        impl Debug;

        /// TXVGA2 gain control, log-linear control. LSB=1dB, encoded as shown below.
        /// Code  | Gain [dB]
        /// ------|------------
        /// 00000 | 0 **(Default)**
        /// 00001 | 1
        /// 11001 | 25 ...
        /// 11111 | 25
        pub vga2gain, set_vga2gain: 7, 3;

        /// Selects the signal for AC coupling, MUX provides:
        /// - 0: Reference DC generated inside the selected detector. **(Default)**
        /// - 1: Average of the selected detector output.
        pub envd_sig, set_envd_sig: 2;

        /// Detector select, MUX provides
        /// - 00: AUXPA envelop detector output **(Default)**
        /// - 01: AUXPA peak detector output.
        /// - 10: PA1 envelop detector output.
        /// - 11: PA2 envelop detector output.
        pub envd_det, set_envd_det: 1, 0;
    }
    lmsreg!(TxRf0x45, 0x45);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxRf0x46(u8);
        impl Debug;

        /// Controls the bandwidth of the envelop and peak detectors.
        /// - 0000: Minimum bandwidth, envelop ~1MHz, peak 30kHz. **(Default)**
        /// - 1111: Maximum bandwidth, envelop ~15MHz, peak ~300KHz.
        pub pkdbw, set_pkdbw: 7, 4;

        /// Base band loopback switches control.
        /// - 00: Switch open. **(Default)**
        /// - 11: Switch closed.
        pub loopbben, set_loopbben: 3, 2;

        /// Shorts the resistor in the envelop/peak detector to speed up charging for faster response.
        ///
        /// After the initial charge up, it should be disabled to achieve a LPF function.
        ///
        /// - 0: Switch open, LPF function in effect. **(Default)**
        /// - 1: Resistor shorted (no LPF function).
        pub fst_pkdet, set_fst_pkdet: 1;

        /// Bias stage of high frequency TX part has
        /// large resistors to filter the noise.
        ///
        /// However, they create large settling time. This switch can
        /// be used to short those resistors during the initialization
        /// and then it may be needed to open it to filter the noise,
        /// in case the noise is too high.
        ///
        /// - 0: Switch open (noise filtering functional). **(Default)**
        /// - 1: Resistors shorted (short settling - no noise filtering).
        pub fst_txhfbias, set_fst_txhfbias: 0;
    }
    lmsreg!(TxRf0x46, 0x46);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxRf0x47(u8);
        impl Debug;

        /// Controls the bias current of the LO buffer.
        ///
        /// Higher current will increase the linearity. LSB=5/6mA.
        /// - 0000: Minimum current.
        /// - 0110: TXMIX takes 5mA for buffer. **(Default)**
        /// - 1111: Maximum current.
        pub ict_txlobuf, set_ict_txlobuf: 7, 4;

        /// The linearity of PAs depends on the bias at the base of the cascode NPNs in the PA cells.
        ///
        /// Increasing the VBCAS will lower the base of the cascode NPN.
        ///
        /// - 0000: Maximum base voltage. **(Default)**
        /// - 1111: Minimum base voltage.
        pub vbcas_txdrv, set_vbcas_txdrv: 3, 0;
    }
    lmsreg!(TxRf0x47, 0x47);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxRf0x48(u8);
        impl Debug;

        /// Controls the bias current of the mixer.
        ///
        /// Higher current will increase the linearity. LSB=1mA.
        /// - 00000: 0mA.
        /// - 01100: TXMIX takes 12mA for each cell. **(Default)**
        /// - 11111: 31mA.
        pub ict_txmix, set_ict_txmix: 4, 0;
    }
    lmsreg!(TxRf0x48, 0x48);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxRf0x49(u8);
        impl Debug;
        /// Controls the bias current of the PAs.
        ///
        /// Higher current will increase the linearity. LSB=1mA.
        /// - 00000: 0mA.
        /// - 01100: PAs take 12mA for each cell. **(Default)**
        /// - 11111: 31mA.
        pub ict_txdrv, set_ict_txdrv: 4, 0;
    }
    lmsreg!(TxRf0x49, 0x49);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxRf0x4A(u8);
        impl Debug;

        /// VGA1, I channel power control.
        /// - 0: Powered down.
        /// - 1: Powered up. **(Default)**
        pub pw_vga1_i, set_pw_vga1_i: 4;

        ///  VGA1, Q channel power control.
        /// - 0: Powered down.
        /// - 1: Powered up. **(Default)**
        pub pw_vga1_q, set_pw_vga1_q: 3;

        ///  Power down for PAs and AUXPA.
        /// - 0: PA1, PA2 and AUXPA can be separately controlled. **(Default)**
        /// - 1: PA1, PA2 and AUXPA all disabled
        pub pd_txdrv, set_pd_txdrv: 2;

        ///  Power down for TXLOBUF.
        /// - 0: Powered up. **(Default)**
        /// - 1: Powered down.
        pub pd_txlobuf, set_pd_txlobuf: 1;

        /// Power down for TXMIX.
        /// - 0: Powered up. **(Default)**
        /// - 1: Powered down.
        pub pd_txmix, set_pd_txmix: 0;
    }
    lmsreg!(TxRf0x4A, 0x4A);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxRf0x4B(u8);
        impl Debug;

        /// TXVGA1 gain control, raw access. LSB=1dB, encoded as shown below.
        ///
        /// Code  | Gain [dB]
        /// ------|--------------
        /// 00000 | -35
        /// 00001 | -34
        /// …a    | …a
        /// 10101 | -14 **(Default)**
        /// …a    | …a
        /// 11110 | -5
        /// 11111 | -4
        pub vga1gaint, set_vga1gaint: 7, 0;
    }
    lmsreg!(TxRf0x4B, 0x4B);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxRf0x4C(u8);
        impl Debug;

        /// Controls the gain of PA1, PA2 and AUXPA, raw access.
        /// - For PA1, PA2:
        /// - For AUXPA: Only 4 LSBs are used, max gain ~22dB.
        pub g_txvga2, set_g_txvga2: 7, 0;
    }
    lmsreg!(TxRf0x4C, 0x4C);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct TxRf0x4D(u8);
        impl Debug;

        /// Power down for envelop/peak detectors.
        /// - 0: Powered up. **(Default)**
        /// - 1: Powered down.
        pub pd_pkdet, set_pd_pkdet: 7;
    }
    lmsreg!(TxRf0x4D, 0x4D);
}

mod adcdac {
    ////////////////////////////////////////////////////////////////////////
    // RX LPF, DAC, ADC Registers                                         //
    ////////////////////////////////////////////////////////////////////////
    pub use super::*;

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxLpfDacAdc0x50(u8);
        impl Debug;

        /// Value from DC Calibration module, selected by DC_ADDR.
        pub dc_regval, _: 5, 0;
    }
    lmsreg!(RxLpfDacAdc0x50, 0x50);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxLpfDacAdc0x51(u8);
        impl Debug;

        /// Lock pattern register.
        ///
        /// Locked, when register value is neither "000" nor "111".
        pub dc_lock, _: 4, 2;

        /// Indicates calibration status.
        /// - 1: Calibration in progress.
        /// - 0: Calibration is done.
        pub dc_clbr_done, _: 1;

        /// Value from DC module comparator, selected by DC_ADDR.
        /// - 1: Count Up.
        /// - 0: Count Down.
        pub dc_ud, _: 0;
    }
    lmsreg!(RxLpfDacAdc0x51, 0x51);
    dc_cal_status_reg!(RxLpfDacAdc0x51);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxLpfDacAdc0x52(u8);
        impl Debug;

        /// Value to load into selected (by DC_ADDR) DC calibration module.
        pub dc_cntval, set_dc_cntval: 5, 0;
    }
    lmsreg!(RxLpfDacAdc0x52, 0x52);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxLpfDacAdc0x53(u8);
        impl Debug;

        /// Start calibration command of the module, selected by DC_ADDR.
        /// - 1: Start Calibration.
        /// - 0: Deactivate Start Calibration command. **(Default)**
        pub dc_start_clbr, set_dc_start_clbr: 5;

        /// Load value from DC_CNTVAL to module, selected by DC_ADDR.
        /// - 1: Load Value.
        /// - 0: Deactivate Load Value command. **(Default)**
        pub dc_load, set_dc_load: 4;

        /// Resets all DC Calibration modules.
        /// - 1: Reset inactive. **(Default)**
        /// - 0: Reset active.
        pub dc_sreset, set_dc_sreset: 3;

        /// Active calibration module address.
        /// - 000: I filter. **(Default)**
        /// - 001: Q filter.
        /// 010-- 111: Not used.
        pub dc_addr, set_dc_addr: 2, 0;
    }
    lmsreg!(RxLpfDacAdc0x53, 0x53);
    dc_cal_control_reg!(RxLpfDacAdc0x53);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxLpfDacAdc0x54(u8);
        impl Debug;

        /// LPF bandwidth control.
        ///
        /// code | Bandwidth [MHz]
        /// -----|----------------
        /// 0000 | 14 **(Default)**
        /// 0001 | 10
        /// 0010 | 7
        /// 0011 | 6
        /// 0100 | 5
        /// 0101 | 4.375
        /// 0110 | 3.5
        /// 0111 | 3
        /// 1000 | 2.75
        /// 1001 | 2.5
        /// 1010 | 1.92
        /// 1011 | 1.5
        /// 1100 | 1.375
        /// 1101 | 1.25
        /// 1110 | 0.875
        /// 1111 | 0.75
        pub bwc_lpf, set_bwc_lpf: 5, 2;

        /// LPF modules enable.
        /// - 0: LPF modules powered down.
        /// - 1: LPF modules enabled. **(Default)**
        pub en, set_en: 1;

        /// Decode.
        /// - 0: Decode control signals. **(Default)**
        /// - 1: Use control signals from test mode registers.
        pub decode, set_decode: 0;
    }
    lmsreg!(RxLpfDacAdc0x54, 0x54);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxLpfDacAdc0x55(u8);
        impl Debug;

        /// BYP_EN_LPF: LPF bypass enable.
        /// - 1: Bypass switches will bypass the LPF.
        /// - 0: Normal operation. **(Default)**
        pub byp_en_lpf, set_byp_en_lpf: 6;

        /// Resistor calibration control for the DC offset cancellation DAC.
        /// - 001100: **(Default)**
        pub dco_daccal, set_dco_daccal: 5, 0;
    }
    lmsreg!(RxLpfDacAdc0x55, 0x55);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxLpfDacAdc0x57(u8);
        impl Debug;

        /// ADC/DAC modules enable.
        /// - 0: ADC/DAC modules powered down.
        /// - 1: ADC/DAC modules enabled. **(Default)**
        pub en_adc_dac, set_en_adc_dac: 7;

        /// Decode.
        /// - 0: Decode ADC/DAC enable signals. **(Default)**
        /// - 1: Use ADC/DAC enable signals from MISC_CTRL[4:0] register.
        pub decode, set_decode: 6;

        /// DAC Internal Output Load Resistor Control Bits.
        /// - 111: 50 Ohms.
        /// - 110: 100 Ohms.
        /// - 101: 66 Ohms.
        /// - 100: 200 Ohms.
        /// - 011: 66 Ohms.
        /// - 010: 200 Ohms. **(Default)**
        /// - 001: 100 Ohms.
        /// - 000: Open Circuit.
        pub tx_ctrl1_out_load_res_ctl, set_tx_ctrl1_out_load_res_ctl: 5, 3;

        /// DAC Reference Current Resistor.
        /// - 1: External. **(Default)**
        /// - 0: Internal.
        pub tx_ctrl1_refres, set_tx_ctrl1_refres: 2;

        /// DAC Full Scale Output Current Control (single-ended).
        /// - 11: Iout FS=5ma.
        /// - 10: Iout FS=2.5ma.
        /// - 01: Iout FS=10ma.
        /// - 00: Iout FS=5ma. **(Default)**
        pub tx_ctrl1_fso, set_tx_ctrl1_fso: 1, 0;
    }
    lmsreg!(RxLpfDacAdc0x57, 0x57);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxLpfDacAdc0x56(u8);
        impl Debug;

        /// Calibration value, coming from TRX_LPF_CAL module.
        /// - 011: **(Default)**
        pub rccal_lpf, set_rccal_lpf: 6, 4;

        /// Power down for the DAC in the DC offset cancellation block.
        /// - 1: Powered Down.
        /// - 0: Enabled. **(Default)**
        pub pd_dcodac_lpf, set_pd_dcodac_lpf: 2;

        /// Power down signal for the dc_ref_con3 block.
        /// - 1: Powered Down.
        /// - 0: Enabled. **(Default)**
        pub pd_dcoref_lpf, set_pd_dcoref_lpf: 1;

        /// Power down for the filter.
        /// - 1: Powered Down.
        /// - 0: Enabled. **(Default)**
        pub pd_fil_lpf, set_pd_fil_lpf: 0;
    }
    lmsreg!(RxLpfDacAdc0x56, 0x56);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxLpfDacAdc0x58(u8);
        impl Debug;

        /// Reference bias resistor adjust.
        /// - 11: 15uA.
        /// - 10: 10uA.
        /// - 01: 40uA.
        /// - 00: 20uA. **(Default)**
        pub rx_ctrl1_ref_bias_res_adj, set_rx_ctrl1_ref_bias_res_adj: 7, 6;

        /// Reference bias UP.
        /// - 11: 2.5X.
        /// - 10: 2.0X.
        /// - 01: 1.5X.
        /// - 00: 1.0X. **(Default)**
        pub rx_ctrl1_ref_bias_up, set_rx_ctrl1_ref_bias_up: 5, 4;

        /// Reference bias DOWN.
        /// - 1111: Min bias.
        /// - …
        /// - 0000: Max bias. **(Default)**
        pub rx_ctrl1_ref_bias_down, set_rx_ctrl1_ref_bias_down: 3, 0;
    }
    lmsreg!(RxLpfDacAdc0x58, 0x58);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxLpfDacAdc0x59(u8);
        impl Debug;

        /// Reference Gain Adjust.
        /// - 11: 1.25V.
        /// - 10: 1.00V.
        /// - 01: 1.75V.
        /// - 00: 1.50V. **(Default)**
        pub rx_ctrl2_ref_gain_adj, set_rx_ctrl2_ref_gain_adj: 6, 5;

        /// Common Mode Adjust.
        /// - 11: 790mV
        /// - 10: 700mV
        /// - 01: 960mV
        /// - 00: 875mV **(Default)**
        pub rx_ctrl2_common_mode_adj, set_rx_ctrl2_common_mode_adj: 4, 3;

        /// Reference Buffer Boost.
        /// - 11: 2.5X.
        /// - 10: 2.0X.
        /// - 01: 1.5X.
        /// - 00: 1.0X. **(Default)**
        pub rx_ctrl2_reg_buf_boost, set_rx_ctrl2_reg_buf_boost: 2, 1;

        /// ADC Input Buffer Disable.
        /// - 1: Disabled. **(Default)**
        /// - 0: Enabled.
        pub rx_ctrl2_adc_input_buf_dis, set_rx_ctrl2_adc_input_buf_dis: 0;
    }
    lmsreg!(RxLpfDacAdc0x59, 0x59);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxLpfDacAdc0x5A(u8);
        impl Debug;

        /// Rx Fsync Polarity, frame start.
        /// - 1: 1.
        /// - 0: 0. **(Default)**
        pub misc_ctrl_rx_fsync_pol, set_misc_ctrl_rx_fsync_pol: 7;

        /// Rx Interleave Mode.
        /// - 1: Q,I.
        /// - 0: I,Q. **(Default)**
        pub misc_ctrl_rx_interleave_mode, set_misc_ctrl_rx_interleave_mode: 6;

        /// DAC Clk Edge Polarity.
        /// - 1: Negative. **(Default)**
        /// - 0: Positive.
        pub misc_ctrl_dac_clk_edge_pol, set_misc_ctrl_dac_clk_edge_pol: 5;

        /// Tx Fsync Polarity, frame start.
        /// - 1: 1.
        /// - 0: 0. **(Default)**
        pub misc_ctrl_tx_fsync_pol, set_misc_ctrl_tx_fsync_pol: 4;

        /// Tx Interleave Mode.
        /// - 1: Q,I.
        /// - 0: I,Q. **(Default)**
        pub misc_ctrl_tx_interleave_mode, set_misc_ctrl_tx_interleave_mode: 3;

        /// ADC Sampling Phase Select.
        /// - 1: Falling edge.
        /// - 0: Rising edge. **(Default)**
        pub rx_ctrl3_adc_sampling_phase, set_rx_ctrl3_adc_sampling_phase: 2;

        /// Clock Non-Overlap Adjust.
        /// - 11: +300ps.
        /// - 10: +150ps.
        /// - 01: +450ps.
        /// - 00: Nominal. **(Default)**
        pub rx_ctrl3_clk_non_overlap_adj, set_rx_ctrl3_clk_non_overlap_adj: 1, 0;
    }
    lmsreg!(RxLpfDacAdc0x5A, 0x5A);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxLpfDacAdc0x5B(u8);
        impl Debug;

        /// ADC bias resistor adjust.
        /// - 11: 15uA.
        /// - 10: 10uA.
        /// - 01: 40uA.
        /// - 00: 20uA. **(Default)**
        pub rx_ctrl4_adc_bias_res_adj, set_rx_ctrl4_adc_bias_res_adj: 7, 6;

        /// Main bias DOWN.
        /// - 11: Min bias.
        /// - 10:
        /// - 01:
        /// - 00: Nominal. **(Default)**
        pub rx_ctrl4_main_bias_down, set_rx_ctrl4_main_bias_down: 5, 4;

        /// ADC Amp1 stage1 bias UP.
        /// - 11: 15uA.
        /// - 10: 10uA.
        /// - 01: 40uA.
        /// - 00: 20uA. **(Default)**
        pub rx_ctrl4_amp1_stage1_bias_up, set_rx_ctrl4_amp1_stage1_bias_up: 3, 2;

        /// ADC Amp2-4 stage1 bias UP.
        /// - 11: 15uA.
        /// - 10: 10uA.
        /// - 01: 40uA.
        /// - 00: 20uA. **(Default)**
        pub rx_ctrl4_adc_amp2_4_stage1_bias_up, set_rx_ctrl4_adc_amp2_4_stage1_bias_up: 1, 0;
    }
    lmsreg!(RxLpfDacAdc0x5B, 0x5B);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxLpfDacAdc0x5C(u8);
        impl Debug;

        /// ADC Amp1 stage2 bias UP.
        /// - 11: 15uA.
        /// - 10: 10uA.
        /// - 01: 40uA.
        /// - 00: 20uA. **(Default)**
        pub rx_ctrl5_adc_amp1_stage2_bias_up, set_rx_ctrl5_adc_amp1_stage2_bias_up: 7, 6;

        /// ADC Amp2-4 stage2 bias UP.
        /// - 11: 15uA.
        /// - 10: 10uA.
        /// - 01: 40uA.
        /// - 00: 20uA. **(Default)**
        pub rx_ctrl5_adc_amp2_4_stage2_bias_up, set_rx_ctrl5_adc_amp2_4_stage2_bias_up: 5, 4;

        /// Quantizer bias UP.
        /// - 11: 15uA.
        /// - 10: 10uA.
        /// - 01: 40uA.
        /// - 00: 20uA. **(Default)**
        pub rx_ctrl5_quat_bias_up, set_rx_ctrl5_quat_bias_up: 3, 2;

        /// Input Buffer bias UP.
        /// - 11: 15uA.
        /// - 10: 10uA.
        /// - 01: 40uA.
        /// - 00: 20uA. **(Default)**
        pub rx_ctrl5_inbuf_bias_up, set_rx_ctrl5_inbuf_bias_up: 1, 0;
    }
    lmsreg!(RxLpfDacAdc0x5C, 0x5C);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxLpfDacAdc0x5D(u8);
        impl Debug;

        /// Bandgap Temperature Coefficient Control.
        /// - 0111: Max.
        /// - 0000: Nominal. **(Default)**
        /// - 1000: Min.
        pub ref_ctrl0_bandgap_temp_coef, set_ref_ctrl0_bandgap_temp_coef: 7, 4;

        /// Bandgap Gain Control.
        /// - 0111: Max.
        /// - 0000: Nominal. **(Default)**
        /// - 1000: Min.
        pub ref_ctrl0_bandgap_gain, set_ref_ctrl0_bandgap_gain: 3, 0;
    }
    lmsreg!(RxLpfDacAdc0x5D, 0x5D);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxLpfDacAdc0x5E(u8);
        impl Debug;

        /// Reference Amps bias adjust.
        /// - 11: 15uA.
        /// - 10: 10uA.
        /// - 01: 40uA.
        /// - 00: 20uA. **(Default)**
        pub ref_ctrl1_reg_amps_bias_adj, set_ref_ctrl1_reg_amps_bias_adj: 7, 6;

        /// Reference Amps bias UP.
        /// - 11: 2.5X.
        /// - 10: 2.0X.
        /// - 01: 1.5X.
        /// - 00: 1.0X. **(Default)**
        pub ref_ctrl1_reg_amps_bias_up, set_ref_ctrl1_reg_amps_bias_up: 5, 4;

        /// Reference Amps bias DOWN.
        /// - 1111: Min bias.
        /// - …
        /// - 0000: Max bias. **(Default)**
        pub ref_ctrl1_reg_amps_bias_down, set_ref_ctrl1_reg_amps_bias_down: 3, 0;
    }
    lmsreg!(RxLpfDacAdc0x5E, 0x5E);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxLpfDacAdc0x5F(u8);
        impl Debug;

        /// Power down DC offset comparators in DC offset cancellation block. Should be powered up only when DC offset cancellation algorithm is running.
        /// - 1: Powered Down.
        /// - 0: Enabled. **(Default)**
        pub pd_dcocmp_lpf, set_pd_dcocmp_lpf: 7;

        /// Enable DAC.
        /// - 1: Enable. **(Default)**
        /// - 0: Off.
        pub misc_ctrl_dac_en, set_misc_ctrl_dac_en: 4;

        /// Enable ADC1 (I Channel).
        /// - 1: Enable. **(Default)**
        /// - 0: Off.
        pub misc_ctrl_adc1_en, set_misc_ctrl_adc1_en: 3;

        /// Enable ADC2 (Q Channel).
        /// - 1: Enable. **(Default)**
        /// - 0: Off.
        pub misc_ctrl_adc2_en, set_misc_ctrl_adc2_en: 2;

        /// Enable ADC reference.
        /// - 1: Enable. **(Default)**
        /// - 0: Off.
        pub misc_ctrl_adc_ref_en, set_misc_ctrl_adc_ref_en: 1;

        /// Enable master reference.
        /// - 1: Enable. **(Default)**
        /// - 0: Off.
        pub misc_ctrl_master_ref_en, set_misc_ctrl_master_ref_en: 0;
    }
    lmsreg!(RxLpfDacAdc0x5F, 0x5F);

}

mod rxvga2 {
    ////////////////////////////////////////////////////////////////////////
    // RX VGA2 Register                                                   //
    ////////////////////////////////////////////////////////////////////////
    use super::*;

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxVga0x60(u8);
        impl Debug;

        /// Value from DC Calibration module selected by DC_ADDR.
        pub dc_regval, _: 5, 0;
    }
    lmsreg!(RxVga0x60, 0x60);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxVga0x61(u8);
        impl Debug;

        /// Lock pattern register.
        ///
        /// Locked when register value is not "000" nor "111".
        pub dc_lock, _: 4, 2;

        /// Indicates calibration status.
        /// - 1: Calibration in progress.
        /// - 0: Calibration is done.
        pub dc_clbr_done, _: 1;

        /// Value from DC module comparator, selected by DC_ADDR
        /// - 1: Count Up.
        /// - 0: Count Down.
        pub dc_ud, _: 0;
    }
    lmsreg!(RxVga0x61, 0x61);
    dc_cal_status_reg!(RxVga0x61);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxVga0x62(u8);
        impl Debug;

        /// Value to load into selected (by DC_ADDR) DC calibration module.
        pub dc_cntval, set_dc_cntval: 5, 0;

    }
    lmsreg!(RxVga0x62, 0x62);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxVga0x63(u8);
        impl Debug;

        /// Start calibration command of the module, selected by DC_ADDR.
        // - 1: Start Calibration.
        // - 0: Deactivate Start Calibration command. **(Default)**
        pub dc_start_clbr, set_dc_start_clbr: 5;

        /// Load value from DC_CNTVAL to module, selected by DC_ADDR.
        // - 1: Load Value.
        // - 0: Deactivate Load Value command. **(Default)**
        pub dc_load, set_dc_load: 4;

        /// Resets all DC Calibration modules.
        /// - 1: Reset inactive. **(Default)**
        /// - 0: Reset active.
        pub dc_sreset, set_dc_sreset: 3;

        /// Active calibration module address.
        /// - 000: DC reference module.
        /// - 001: First gain stage (VGA2A), I channel.
        /// - 010: First gain stage (VGA2A), Q channel.
        /// - 011: Second gain stage (VGA2B), I channel.
        /// - 100: Second gain stage (VGA2B), Q channel.
        /// - 101-111: Not used.
        pub dc_addr, set_dc_addr: 2, 0;
    }
    lmsreg!(RxVga0x63, 0x63);
    dc_cal_control_reg!(RxVga0x63);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxVga0x64(u8);
        impl Debug;

        /// RXVGA2 output common mode voltage control. VCM[3] – sign, VCM[2:0] – magnitude, LSB=40mV.
        ///
        /// Code | Voltage [V]
        /// -----|------------
        /// 0000 | 1.18
        /// 0001 | 1.14
        /// 0010 | 1.10
        /// 0011 | 1.06
        /// 0100 | 1.02
        /// 0101 | 0.98
        /// 0110 | 0.94
        /// 0111 | 0.90 **(Default)**
        /// 1000 | 0.62
        /// 1001 | 0.66
        /// 1010 | 0.70
        /// 1011 | 0.74
        /// 1100 | 0.78
        /// 1101 | 0.82
        /// 1110 | 0.86
        pub vcm, set_vcm: 5, 2;

        /// RXVGA2 modules enable.
        /// - 0: RXVGA2 modules powered down.
        /// - 1: RXVGA2 modules enabled. **(Default)**
        pub en, set_en: 1;

        /// - 0: Decode control signals. **(Default)**
        /// - 1: Use control signals from test mode registers.
        pub decode, set_decode: 0;
    }
    lmsreg!(RxVga0x64, 0x64);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxVga0x65(u8);
        impl Debug;
        /// RXVGA2 gain control.
        ///
        /// LSB=3dB, encoded as shown below.
        ///
        /// Code  | Gain [dB]
        /// ------|------------
        /// 00000 | 0
        /// 00001 | 3 **(Default)**
        /// …     | …
        /// 01001 | 27
        /// 01010 | 30
        /// …     | …
        /// 10100 | 60
        ///
        /// Not recommended to be used above 30dB.
        pub vga2gain, set_vga2gain: 4, 0;
    }
    lmsreg!(RxVga0x65, 0x65);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxVga0x66(u8);
        impl Debug;

        /// DC current regulator.
        /// - 1: Powered down.
        /// - 0: Powered up. **(Default)**
        pub pd9, set_pd_9: 5;

        /// DC calibration DAC for VGA2B.
        /// - 1: Powered down.
        /// - 0: Powered up. **(Default)**
        pub pd8, set_pd8: 4;

        /// DC calibration DAC for VGA2A.
        /// - 1: Powered down.
        /// - 0: Powered up. **(Default)**
        pub pd6, set_pd6: 2;

        /// Band gap.
        /// - 1: Powered down.
        /// - 0: Powered up. **(Default)**
        pub pd4, set_pd4: 0;
    }
    lmsreg!(RxVga0x66, 0x66);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxVga0x67(u8);
        impl Debug;

        /// Output buffer in both RXVGAs.
        /// - 1: Powered down.
        /// - 0: Powered up. **(Default)**
        pub pd3, set_pd3: 3;

        /// RXVGA2B.
        /// - 1: Powered down.
        /// - 0: Powered up. **(Default)**
        pub pd2, set_pd2: 2;

        /// RXVGA2A.
        /// - 1: Powered down.
        /// - 0: Powered up. **(Default)**
        pub pd1, set_pd1: 1;

        /// Current reference.
        /// - 1: Powered down.
        /// - 0: Powered up. **(Default)**
        pub pd0, set_pd0: 0;
    }
    lmsreg!(RxVga0x67, 0x67);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxVga0x68(u8);
        impl Debug;

        /// Controls the gain of second VGA2 stage (VGA2B). LSB=3dB, encoded as shown below.
        /// Code | Gain [dB]
        /// -----|------------
        /// 0000 | 0 **(Default)**
        /// 0001 | 3
        /// …    | …
        /// 1001 | 27
        /// 1010 | 30
        pub vga2gainb, set_vga2gainb: 7, 4;

        /// Controls the gain of first VGA2 stage (VGA2A). LSB=3dB, encoded as shown below.
        /// Code | Gain [dB]
        /// -----|------------
        /// 0000 | 0
        /// 0001 | 3 **(Default)**
        /// …    | …
        /// 1001 | 27
        /// 1010 | 30
        pub vga2gaina, set_vga2gaina: 3, 0;

    }
    lmsreg!(RxVga0x68, 0x68);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxVga0x6E(u8);
        impl Debug;

        /// DC calibration comparator for VGA2B.
        /// - 1: Powered down.
        /// - 0: Powered up. **(Default)**
        pub pd7, set_pd7: 7;

        /// DC calibration comparator for VGA2A.
        /// - 1: Powered down.
        /// - 0: Powered up. **(Default)**
        pub pd6, set_pd6: 6;
    }
    lmsreg!(RxVga0x6E, 0x6E);
}

mod rxfe {
    ////////////////////////////////////////////////////////////////////////
    // RX Frontend Registers                                              //
    ////////////////////////////////////////////////////////////////////////
    use super::*;

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxFe0x70(u8);
        impl Debug;

        /// DECODE
        /// - 0: Decode control signals. **(Default)**
        /// - 1: Use control signals from test mode registers.
        pub decode, set_decode: 1;

        /// EN: RXFE modules enable.
        /// - 0: Top modules powered down
        /// - 1: Top modules enabled **(Default)**
        pub en, set_en: 0;
    }
    lmsreg!(RxFe0x70, 0x70);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxFe0x71(u8);
        impl Debug;

        /// Selects the input to the mixer.
        /// - 1: Input 1 is selected, shorted on-chip to LNA internal output. **(Default)**
        /// - 0: Input 2 is selected, connected to pads.
        pub in1sel_mix_rxfe, set_in1sel_mix_rxfe: 7;

        /// DC offset cancellation, I channel.
        /// - Code is Sign(<6>)-Magnitude(<5:0>), signed magnitude format.
        /// - 0000000: **(Default)**
        pub dcoff_i_rxfe, set_dcoff_i_rxfe: 6, 0;
    }
    lmsreg!(RxFe0x71, 0x71);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxFe0x72(u8);
        impl Debug;

        /// To select the internal load for the LNA.
        /// - 1: Internal load is active. **(Default)**
        /// - 0: Internal load is disabled.
        pub inload_lna_rxfe, set_inload_lna_rxfe: 7;

        /// DC offset cancellation, Q channel.
        /// - Code is Sign(<6>)-Magnitude(<5:0>), signed magnitude format.
        /// - 0000000: **(Default)**
        pub dcoff_q_rxfe, set_dcoff_q_rxfe: 6, 0;

    }
    lmsreg!(RxFe0x72, 0x72);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxFe0x73(u8);
        impl Debug;

        /// To select the external load for the LNA.
        /// - 1: External load is active.
        /// - 0: External load is disabled. **(Default)**
        pub xload_lna_rxfe, set_xload_lna_rxfe: 7;

        /// IP2 cancellation, I channel.
        /// - Code is Sign(<6>)-Magnitude(<5:0>), signed magnitude format.
        /// - 0000000: **(Default)**
        pub ip2trim_i_rxfe, set_ip2trim_i_rxfe: 6, 0;
    }
    lmsreg!(RxFe0x73, 0x73);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxFe0x75(u8);
        impl Debug;

        /// LNA gain mode control.
        /// - 11: Max gain (all LNAs). **(Default)**
        /// - 10: Mid gain (all LNAs).
        /// - 01: LNA bypassed (LNA1 and LNA2).
        /// - 00: Max gain (LNA3).
        pub g_lna_rxfe, set_g_lna_rxfe: 7, 6;

        /// Selects the active LNA.
        /// - 00: All LNAs disabled.
        /// - 01: LNA1 active. **(Default)**
        /// - 10: LNA2 active.
        /// - 11: LNA3 active.
        pub lnasel_rxfe, set_lnasel_rxfe: 5, 4;

        /// Controls the capacitance parallel to the BE of the input NPN transistors.
        ///
        /// To be used at lower frequencies for easier matching.
        ///
        /// **NOTE**: For LNA1 and LNA2 only.
        /// - 0000: **(Default)**
        pub cbe_lna_rxfe, set_cbe_lna_rxfe: 3, 0;

    }
    lmsreg!(RxFe0x75, 0x75);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxFe0x76(u8);
        impl Debug;

        /// Feedback resistor control of the TIA (RXVGA1) to set the mixer gain.
        /// - If = 120 --> mixer gain = 30dB **(Default)**
        /// - If = 102 --> mixer gain = 19dB
        /// - If = 2 --> mixer gain = 5dB
        pub rfb_tia_rxfe, set_rfb_tia_rxfe: 6, 0;
    }
    lmsreg!(RxFe0x76, 0x76);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxFe0x77(u8);
        impl Debug;

        /// Feedback capacitor for the TIA (RXVGA1) to limit the BW.
        /// - If = 0, min cap --> BW~45MHz for gain of 30dB. **(Default)**
        /// - If = 19 --> BW=2.5MHz for MixGain=30dB and at TT.
        ///
        /// This cap is supposed to be set according to the RC time
        /// constant to have almost constant BW over the corners for
        /// optimum CDMA performance. Software will control it using
        /// the information from the LPF calibration circuit.
        pub cfb_tia_rxfe, set_cfb_tia_rxfe: 6, 0;
    }
    lmsreg!(RxFe0x77, 0x77);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxFe0x78(u8);
        impl Debug;

        /// Controls the on-chip LNA load resistor for the external load mode of the LNA.
        ///
        /// In practice, this will be set to high value, the output
        /// will be ac coupled, and the actual load is defined on PCB.
        ///
        /// - 011100: **(Default)**
        pub rdlext_lna_rxfe, set_rdlext_lna_rxfe: 5, 0;

    }
    lmsreg!(RxFe0x78, 0x78);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxFe0x79(u8);
        impl Debug;

        /// Controls the on-chip LNA load resistor for the internal
        /// load mode of the LNA, LNA1 and LNA2.
        /// - 011100: **(Default)**
        pub rdlint_lna_rxfe, set_rdlint_lna_rxfe: 5, 0;
    }
    lmsreg!(RxFe0x79, 0x79);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxFe0x7A(u8);
        impl Debug;

        /// Control for tweaking the bias current for mixer.
        /// - 0000: 0 bias current.
        /// - 0111: nominal bias current. **(Default)**
        /// - 1111: 2.1x nominal bias current.
        pub ict_mix_rxfe, set_ict_mix_rxfe: 7, 4;

        /// Control for tweaking the bias current for LNA.
        /// - 0000: 0 bias current.
        /// - 0111: nominal bias current. **(Default)**
        /// - 1111: 2.1x nominal bias current.
        pub ict_lna_rxfe, set_ict_lna_rxfe: 3, 0;

    }
    lmsreg!(RxFe0x7A, 0x7A);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxFe0x7B(u8);
        impl Debug;

        /// Control for tweaking the bias current for TIA (RXVGA1).
        /// - 0000: 0 bias current.
        /// - 0111: nominal bias current. **(Default)**
        /// - 1111: 2.1x nominal bias current.
        pub ict_tia_rxfe, set_ict_tia_rxfe: 7, 4;

        /// Control for tweaking the bias current for mixer LO buffer.
        /// - 0000: 0 bias current.
        /// - 0111: nominal bias current. **(Default)**
        /// - 1111: 2.1x nominal bias current.
        pub ict_mxlob_rxfe, set_ict_mxlob_rxfe: 3, 0;
    }
    lmsreg!(RxFe0x7B, 0x7B);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxFe0x7C(u8);
        impl Debug;

        /// Tweak for the LO bias of the mixer for optimum linearity.
        /// - 0000: Minimum bias voltage.
        /// - 0011: **(Default)**
        /// - 1111: Maximum bias voltage.
        pub lobn_mix_rxfe, set_lobn_mix_rxfe: 6, 3;

        /// Termination resistor on external mixer input enable.
        /// - 1: Active.
        /// - 0: Inactive. **(Default)**
        pub rinen_mix_rxfe, set_rinen_mix_rxfe: 2;

        /// LNA3 fine gain adjustment.
        /// - 00: +0 dB **(Default)**
        /// - 01: +1 dB
        /// - 10: +2 dB
        /// - 11: +3 dB
        pub g_fine_lna3_rxfe, set_g_fine_lna3_rxfe: 1, 0;
    }
    lmsreg!(RxFe0x7C, 0x7C);

    bitfield!{
        #[derive(Clone, Copy)]
        pub struct RxFe0x7D(u8);
        impl Debug;

        /// TIA (RXVGA1) power down.
        /// - 0: Block active. **(Default)**
        /// - 1: Block inactive.
        pub pd_tia_rxfe, set_pd_tia_rxfe: 3;

        /// Mixer LO buffer power down.
        /// - 0: Block active. **(Default)**
        /// - 1: Block inactive.
        pub pd_mxlob_rxfe, set_pd_mxlob_rxfe: 2;

        /// Mixer power down.
        /// - 0: Block active. **(Default)**
        /// - 1: Block inactive.
        pub pd_mix_rxfe, set_pd_mix_rxfe: 1;

        /// LNA power down.
        /// - 0: Block active. **(Default)**
        /// - 1: Block inactive.
        pub pd_lna_rxfe, set_pd_lna_rxfe: 0;
    }
    lmsreg!(RxFe0x7D, 0x7D);
}

// Represents a reserved register.
#[derive(Clone, Copy, Debug)]
struct Reserved(u8);

/// Returns a boxed `Debug` object from a register's `addr` and `val`
/// pair.
///
/// This a weak form of up-casting.
#[cfg(feature = "std")]
pub fn into_debug(addr: u8, val: u8) -> ::error::Result<Box<Debug>> {
    Ok(match addr {
        ////////////////////////////////////////////////////////////////////
        // Top-level                                                      //
        ////////////////////////////////////////////////////////////////////
        0x00 => Box::new(Top0x00(val)),
        0x01 => Box::new(Top0x01(val)),
        0x02 => Box::new(Top0x02(val)),
        0x03 => Box::new(Top0x03(val)),
        0x04 => Box::new(Top0x04(val)),
        0x05 => Box::new(Top0x05(val)),
        0x06 => Box::new(Top0x06(val)),
        0x07 => Box::new(Top0x07(val)),
        0x08 => Box::new(Top0x08(val)),
        0x09 => Box::new(Top0x09(val)),
        0x0A => Box::new(Top0x0A(val)),
        0x0B => Box::new(Top0x0B(val)),
        // 0x0C
        // 0x0D
        // 0x0E
        // 0x0F
        ////////////////////////////////////////////////////////////////////
        // TX PLL                                                         //
        ////////////////////////////////////////////////////////////////////
        0x10 => Box::new(Pll0x00(val)),
        0x11 => Box::new(Pll0x01(val)),
        0x12 => Box::new(Pll0x02(val)),
        0x13 => Box::new(Pll0x03(val)),
        0x14 => Box::new(Pll0x04(val)),
        0x15 => Box::new(Pll0x05(val)),
        0x16 => Box::new(Pll0x06(val)),
        0x17 => Box::new(Pll0x07(val)),
        0x18 => Box::new(Pll0x08(val)),
        0x19 => Box::new(Pll0x09(val)),
        0x1A => Box::new(Pll0x0A(val)),
        0x1B => Box::new(Pll0x0B(val)),
        // 0x1C
        // 0x1D
        // 0x1E
        // 0x1F
        ////////////////////////////////////////////////////////////////////
        // RX PLL                                                         //
        ////////////////////////////////////////////////////////////////////
        0x20 => Box::new(Pll0x00(val)),
        0x21 => Box::new(Pll0x01(val)),
        0x22 => Box::new(Pll0x02(val)),
        0x23 => Box::new(Pll0x03(val)),
        0x24 => Box::new(Pll0x04(val)),
        0x25 => Box::new(Pll0x05(val)),
        0x26 => Box::new(Pll0x06(val)),
        0x27 => Box::new(Pll0x07(val)),
        0x28 => Box::new(Pll0x08(val)),
        0x29 => Box::new(Pll0x09(val)),
        0x2A => Box::new(Pll0x0A(val)),
        0x2B => Box::new(Pll0x0B(val)),
        // 0x2C
        // 0x2D
        // 0x2E
        // 0x2F
        ////////////////////////////////////////////////////////////////////
        // TX LPF                                                         //
        ////////////////////////////////////////////////////////////////////
        0x30 => Box::new(TxLpf0x30(val)),
        0x31 => Box::new(TxLpf0x31(val)),
        0x32 => Box::new(TxLpf0x32(val)),
        0x33 => Box::new(TxLpf0x33(val)),
        0x34 => Box::new(TxLpf0x34(val)),
        0x35 => Box::new(TxLpf0x35(val)),
        0x36 => Box::new(TxLpf0x36(val)),
        // 0x37
        // 0x38
        // 0x39
        // 0x3A
        // 0x3B
        // 0x3C
        // 0x3D
        // 0x3E
        0x3F => Box::new(TxLpf0x3F(val)),
        ////////////////////////////////////////////////////////////////////
        // TX RF                                                          //
        ////////////////////////////////////////////////////////////////////
        0x40 => Box::new(TxRf0x40(val)),
        0x41 => Box::new(TxRf0x41(val)),
        0x42 => Box::new(TxRf0x42(val)),
        0x43 => Box::new(TxRf0x43(val)),
        0x44 => Box::new(TxRf0x44(val)),
        0x45 => Box::new(TxRf0x45(val)),
        0x46 => Box::new(TxRf0x46(val)),
        0x47 => Box::new(TxRf0x47(val)),
        0x48 => Box::new(TxRf0x48(val)),
        0x49 => Box::new(TxRf0x49(val)),
        0x4A => Box::new(TxRf0x4A(val)),
        0x4B => Box::new(TxRf0x4B(val)),
        0x4C => Box::new(TxRf0x4C(val)),
        0x4D => Box::new(TxRf0x4D(val)),
        // 0x4E
        // 0x4F
        ////////////////////////////////////////////////////////////////////
        // RX LPF, DAC, ADC                                               //
        ////////////////////////////////////////////////////////////////////
        0x50 => Box::new(RxLpfDacAdc0x50(val)),
        0x51 => Box::new(RxLpfDacAdc0x51(val)),
        0x52 => Box::new(RxLpfDacAdc0x52(val)),
        0x53 => Box::new(RxLpfDacAdc0x53(val)),
        0x54 => Box::new(RxLpfDacAdc0x54(val)),
        0x55 => Box::new(RxLpfDacAdc0x55(val)),
        0x56 => Box::new(RxLpfDacAdc0x56(val)),
        0x57 => Box::new(RxLpfDacAdc0x57(val)),
        0x58 => Box::new(RxLpfDacAdc0x58(val)),
        0x59 => Box::new(RxLpfDacAdc0x59(val)),
        0x5A => Box::new(RxLpfDacAdc0x5A(val)),
        0x5B => Box::new(RxLpfDacAdc0x5B(val)),
        0x5C => Box::new(RxLpfDacAdc0x5C(val)),
        0x5D => Box::new(RxLpfDacAdc0x5D(val)),
        0x5E => Box::new(RxLpfDacAdc0x5E(val)),
        0x5F => Box::new(RxLpfDacAdc0x5F(val)),
        ////////////////////////////////////////////////////////////////////
        // RX VGA2                                                        //
        ////////////////////////////////////////////////////////////////////
        0x60 => Box::new(RxVga0x60(val)),
        0x61 => Box::new(RxVga0x61(val)),
        0x62 => Box::new(RxVga0x62(val)),
        0x63 => Box::new(RxVga0x63(val)),
        0x64 => Box::new(RxVga0x64(val)),
        0x65 => Box::new(RxVga0x65(val)),
        0x66 => Box::new(RxVga0x66(val)),
        0x67 => Box::new(RxVga0x67(val)),
        0x68 => Box::new(RxVga0x68(val)),
        // 0x69
        // 0x6A
        // 0x6B
        // 0x6C
        // 0x6D
        0x6E => Box::new(RxVga0x6E(val)),
        // 0x6F
        ////////////////////////////////////////////////////////////////////
        // RX FE                                                          //
        ////////////////////////////////////////////////////////////////////
        0x70 => Box::new(RxFe0x70(val)),
        0x71 => Box::new(RxFe0x71(val)),
        0x72 => Box::new(RxFe0x72(val)),
        0x73 => Box::new(RxFe0x73(val)),
        // 0x74
        0x75 => Box::new(RxFe0x75(val)),
        0x76 => Box::new(RxFe0x76(val)),
        0x77 => Box::new(RxFe0x77(val)),
        0x78 => Box::new(RxFe0x78(val)),
        0x79 => Box::new(RxFe0x79(val)),
        0x7A => Box::new(RxFe0x7A(val)),
        0x7B => Box::new(RxFe0x7B(val)),
        0x7C => Box::new(RxFe0x7C(val)),
        0x7D => Box::new(RxFe0x7D(val)),
        // 0x7E
        // 0x7F
        addr if addr < 128 => Box::new(val),
        _ => return Err(::error::Error::Range),
    })
}

#[cfg(all(test, feature = "std"))]
quickcheck! {
    fn prop_into_debug(addr: u8, val: u8) -> bool {
        let res = into_debug(addr, val);
        let _ = format!("{:?}", res);
        ((addr >= 128) && res.is_err()) || ((addr < 128) && res.is_ok())
    }
}
