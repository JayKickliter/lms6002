#![allow(non_camel_case_types)]

use std::path::PathBuf;
use std::str::FromStr;

/// Will attempt to parse an integer from a hex, binary, or decimal
/// string based on the leading two characters:
/// - `0x`: parse as hex
/// - `0b`: binary
/// - other: decimal
trait FromHexDecBin: Sized {
    fn from_hex_dec_bin(&str) -> Result<Self, ::std::num::ParseIntError>;
}

macro_rules! impl_from_hex_dec_bin {
    ($T:tt) => {
        impl FromHexDecBin for $T {
            fn from_hex_dec_bin(s: &str) -> Result<$T, ::std::num::ParseIntError> {
                if s.len() > 2 {
                    match s.split_at(2) {
                        ("0x", rest) => $T::from_str_radix(rest, 16),
                        ("0b", rest) => $T::from_str_radix(rest, 2),
                        _ => $T::from_str_radix(s, 10),
                    }
                } else {
                    $T::from_str_radix(s, 10)
                }
            }
        }
    };
}

impl_from_hex_dec_bin!(u8);

#[derive(StructOpt, Debug)]
pub struct Opts {
    #[structopt(
        parse(from_os_str),
        short = "d",
        long = "spidev",
        env = "LMS_DEV",
        name = "SPIDEV"
    )]
    /// Path to Spidev, e.g. `/dev/spidev1.0`
    pub dev: PathBuf,

    #[structopt(subcommand)]
    pub cmd: Cmd,
}

/// Top-level commands.
#[derive(Debug, StructOpt)]
pub enum Cmd {
    /// Configure RX PLL
    rxpll(TRxCmd),

    /// Configure TX PLL
    txpll(TRxCmd),

    /// Configure RX LPF
    rxlpf(LpfCmd),

    /// Configure TX LPF
    txlpf(LpfCmd),

    /// Direct register manipulation
    reg(RegCmd),

    /// Calibration
    cal(CalCmd),

    /// Configure RX VGA2
    rxvga2(RxVga2Cmd),
}

/// High-level commands specific to Tx/Rx path
#[derive(Debug, StructOpt)]
pub enum TRxCmd {
    /// Soft-enable this path
    enable,
    /// Soft-disable this path
    disable,
    /// Get or set current frequency
    freq {
        #[structopt(long = "set", name = "FREQ")]
        /// Tune to specified freq
        freq: Option<f64>,
    },
}

/// Represents a range of bits in a `u8` register.
#[derive(Debug, Clone, Copy)]
pub struct BitRange(pub usize, pub usize);

impl FromStr for BitRange {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        // As an example, we are trying to turn, `"7..3"` into
        // `BitRange(7,3)`.

        let err_str = "Bit range must a digit (high bit index), two dots, then another digit (low bit index): '7..3'";

        // There will always be 4 characters in this parameter.
        if s.len() < 4 {
            return Err(err_str.to_owned());
        }

        // The middle two characters will always be `".."`.
        if &s[1..3] != ".." {
            return Err(err_str.to_owned());
        }

        // The first character is always a single decimal digit.
        let high_bit_idx = { usize::from_str(&s[0..1]).or_else(|_| Err(err_str.to_owned()))? };

        // The fourth character is always a single decimal digit.
        let low_bit_idx = { usize::from_str(&s[3..4]).or_else(|_| Err(err_str.to_owned()))? };

        // The high bit index must be `>=` the low bit index.
        if high_bit_idx < low_bit_idx {
            return Err(err_str.to_owned());
        }

        Ok(BitRange(high_bit_idx, low_bit_idx))
    }
}

/// Low-level register command
#[derive(Debug, StructOpt)]
pub enum RegCmd {
    /// Read the value out of register at `addr`
    read {
        #[structopt(parse(try_from_str = "FromHexDecBin::from_hex_dec_bin"))]
        addr: u8,
    },
    /// Write `val` to register at `addr`
    write {
        /// Address of destination register
        #[structopt(
            name = "ADDR",
            parse(try_from_str = "FromHexDecBin::from_hex_dec_bin")
        )]
        addr: u8,
        /// Write VAL to this range of bits in the destination, e.g. `7..3`
        #[structopt(long = "range", name = "RANGE")]
        range: Option<BitRange>,
        /// Value to write to ADDR
        #[structopt(
            name = "VAL",
            parse(try_from_str = "FromHexDecBin::from_hex_dec_bin")
        )]
        val: u8,
    },
}

#[derive(Debug, StructOpt)]
pub struct LpfCmd {
    // Set LPF to specified cutoff freq
    #[structopt(long = "set", name = "FREQ")]
    pub freq: Option<f64>,
}

#[derive(Debug, StructOpt)]
pub enum CalCmd {
    lpftuning,
    rxlpf,
    txlpf,
    rxvga2,
}

#[derive(Debug, StructOpt)]
pub enum RxVga2Cmd {
    gain {
        #[structopt(long = "set", name = "dB")]
        /// Set RXVGA2's to value in [0,3,6,9,...,60]
        set: Option<u32>,
    },
}
