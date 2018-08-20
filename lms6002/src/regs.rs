#![allow(non_camel_case_types)]
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

    PLL0 [
        /// Integer part of the divider (MSBs)
        NINT_8_1 OFFSET(0) NUMBITS(8)
    ],

    PLL1 [
        /// Integer part of the divider (LSB)
        NINT_0 OFFSET(7) NUMBITS(1),
        /// Fractional part of the divider
        NFRAC_22_16 OFFSET(0) NUMBITS(7)
    ],

    PLL2 [
        /// Fractional part of the divider
        NFRAC_15_8 OFFSET(0) NUMBITS(7)
    ],

    PLL3 [
        /// Fractional part of the divider
        NFRAC_7_0 OFFSET(0) NUMBITS(7)
    ],

    PLL4 [
        /// Dithering control
        /// - 0 – disabled
        /// - 1 – enabled (default)
        DITHEN OFFSET(7) NUMBITS(1),
        /// How many bits to dither if DITHEN=1
        /// - 000 – 1 bit (default)
        /// - 001 – 2 bits
        /// - 010 – 3 bits
        /// - ...
        /// - 111 – 8 bits
        DITHN OFFSET(4) NUMBITS(3),
        /// PLL enable
        /// - 0 – PLL powered down
        /// - 1 – PLL enabled (default)
        EN OFFSET(3) NUMBITS(1),
        /// Delta sigma auto bypass when NFRAC = 0
        /// - 0 – disabled (default)
        /// - 1 – enabled
        AUTOBYP OFFSET(2) NUMBITS(1),
        /// - 0 – decode power down/enable signals (default)
        /// - 1 – use power down/enable signals from test mode registers
        DECODE OFFSET(1) NUMBITS(1)
    ]
];
