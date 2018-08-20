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

register_bitfields! [
    // All LMS6002 registers are single-byte.
    u8,

    PLL00 [
        /// Integer part of the divider (MSBs)
        NINT_8_1 OFFSET(0) NUMBITS(8)
    ],

    PLL01 [
        /// Integer part of the divider (LSB)
        NINT_0 OFFSET(7) NUMBITS(1),
        /// Fractional part of the divider
        NFRAC_22_16 OFFSET(0) NUMBITS(7)
    ],

    PLL02 [
        /// Fractional part of the divider
        NFRAC_15_8 OFFSET(0) NUMBITS(7)
    ],

    PLL03 [
        /// Fractional part of the divider
        NFRAC_7_0 OFFSET(0) NUMBITS(7)
    ],

    PLL04 [
        /// Dithering control
        /// - 0: disabled
        /// - 1: enabled (default)
        DITHEN OFFSET(7) NUMBITS(1),
        /// How many bits to dither if DITHEN=1
        /// - 000: 1 bit (default)
        /// - 001: 2 bits
        /// - 010: 3 bits
        /// - ...
        /// - 111: 8 bits
        DITHN OFFSET(4) NUMBITS(3),
        /// PLL enable
        /// - 0: PLL powered down
        /// - 1: PLL enabled (default)
        EN OFFSET(3) NUMBITS(1),
        /// Delta sigma auto bypass when NFRAC = 0
        /// - 0: disabled (default)
        /// - 1: enabled
        AUTOBYP OFFSET(2) NUMBITS(1),
        /// - 0: decode power down/enable signals (default)
        /// - 1: use power down/enable signals from test mode registers
        DECODE OFFSET(1) NUMBITS(1)
    ],

    PLL05 [
        /// VCO selection
        /// - 000: All VCOs powered down
        /// - 100: Low frequency VCO (vco4)
        /// - 101: Mid low frequency VCO (vco3) (default)
        /// - 110: Mid high frequency VCO (vco2)
        /// - 111: High frequency VCO (vco1)
        SELVCO OFFSET(4) NUMBITS(3),
        /// PLL output frequency range selection
        /// - 000: All dividers powered down
        /// - 100: Fvco/2 (2-4GHz range) (default)
        /// - 101: Fvco/4 (1-2GHz range)
        /// - 110: Fvco/8 (0.5-1GHz range)
        /// - 111: Fvco/16 (0.25-0.5GHz range)
        FRANGE OFFSET(2) NUMBITS(3),
        /// Select output buffer in RX PLL, not used in TX PLL
        /// - 00: All output buffers powered down
        /// - 01: First buffer enabled for LNA1 path (default) 10: Second buffer enabled for LNA2 path
        /// - 11: Third buffer enabled for LNA3 path
        SELOUT OFFSET(0) NUMBITS(2)
    ],

    PLL06 [
        /// Enable PFD UP pulses
        /// - 0: disabled
        /// - 1: enabled (default)
        EN_PFD_UP OFFSET(7) NUMBITS(1),
        /// - 0: Test signal output buffer disabled (default)
        /// - 1: Test signal output buffer enabled
        OEN_TSTD_SX OFFSET(6) NUMBITS(1),
        /// - 0: Test signal pass disabled (default)
        /// - 1: Test signal pass enabled
        PASSEN_TSTOD_SD OFFSET(5) NUMBITS(1),
        /// Charge pump current. Binary coded, LSB = 100uA:
        /// - 00000: 0uA
        /// - 00001: 100uA
        /// - ...
        /// - 11000: 2400uA
        /// - ...
        /// - 2400 uA
        ICHP OFFSET(0) NUMBITS(5)
    ],

    PLL07 [
        /// Bypass VCO regulator
        /// - 0: not bypassed
        /// - 1: regulator bypassed (default)
        BYPVCOREG OFFSET(7) NUMBITS(1),
        ///
        /// VCO regulator power down.
        /// - 0: regulator powered up
        /// - 1: regulator powered down (default)
        PDVCOREG OFFSET(6) NUMBITS(1),
        /// VCO regulator band gap settling time control.
        ///
        /// Shorts the resistor in band gap to speed up charging for faster response. After the initial charge up, it should be disabled.
        /// - 1: resistor shorted (default)
        /// - 0: switch open
        FSTVCOBG OFFSET(5) NUMBITS(1),
        /// Charge pump UP offset current. Binary coded, LSB = 10uA:
        /// - 00000: 0uA
        /// - 00001: 10uA
        /// - ...
        /// - 11000: 240uA
        /// - ...: 240uA
        OFFUP OFFSET(0) NUMBITS(5)
    ],

    PLL08 [
        /// VCO regulator output voltage control, 3 MSBs.
        ///
        /// LSB=100mV, VOVCOREG[3:0] coded as below
        /// - 0000: 1.4V, min output
        /// - ...
        /// - 0101: 1.9V (default)
        /// - ...
        /// - 1100: 2.6V, max output
        /// - 1101, 1110, 1111, not valid codes
        VOVCOREG OFFSET(5) NUMBITS(3),
        /// Charge pump DOWN offset current.
        ///
        /// Binary coded, LSB = 10uA:
        /// - 00000: 0uA
        /// - 00001: 10uA
        /// - ...
        /// - 11000: 240uA
        /// - ...: 240uA
        OFFDOWN OFFSET(0) NUMBITS(5)
    ],

    PLL09 [
        /// VCO regulator output voltage control, LSB
        VOVCOREG OFFSET(7) NUMBITS(1),
        /// Switch capacitance programming. Binary coded.
        /// - 000000 (max capacitance, min frequency)
        /// - 010100 (default)
        /// - 111111 (min capacitance, max frequency)
        VCOCAP OFFSET(0) NUMBITS(6)
    ],

    PLL10 [
        /// Value from Vtune comparator (Read Only)
        VTUNE_H OFFSET(7) NUMBITS(1),
        /// Value from Vtune comparator (Read Only)
        VTUNE_L OFFSET(6) NUMBITS(1)
    ],

    PLL11 [
        /// VCO Comparator Enable
        /// - 0: enabled (powered up) (default)
        /// - 1: disabled (powered down)
        PD_VCOCOMP_SX OFFSET(3) NUMBITS(1)
    ]
];

#[cfg(test)]
mod tests {
    use super::*;
    use tock_registers::registers::ReadWrite;
    #[test]
    fn test_pll_regs() {
        let pll11 = ReadWrite::<u8, PLL11::Register>::new(0);
        pll11.write(PLL11::PD_VCOCOMP_SX.val(1));
        assert_eq!(pll11.read(PLL11::PD_VCOCOMP_SX), 1);
        let pll11_val = pll11.get();
        assert_eq!(pll11_val, 1 << 3);
    }
}
