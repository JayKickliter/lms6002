#![allow(non_camel_case_types)]

use std::path::PathBuf;

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
        short = "d",
        long = "spidev",
        env = "LMS_DEV",
        help = "Path to SPI device",
        parse(from_os_str)
    )]
    pub dev: PathBuf,

    #[structopt(subcommand)]
    pub cmd: Cmd,
}

/// Top-level commands.
#[derive(Debug, StructOpt)]
pub enum Cmd {
    /// Configure the Rx path
    rx(TRxCmd),

    /// Configure the Rx path
    tx(TRxCmd),

    /// Direct register manipulation
    reg(RegCmd),
}

/// High-level commands specific to Tx/Rx path
#[derive(Debug, StructOpt)]
pub enum TRxCmd {
    /// Soft-enable this path
    enable,
    /// Soft-disable this path
    disable,
    /// Soft-reset this path
    reset,
    /// Tune to specified frequency
    tune { freq: f64 },
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
        #[structopt(parse(try_from_str = "FromHexDecBin::from_hex_dec_bin"))]
        addr: u8,
        #[structopt(parse(try_from_str = "FromHexDecBin::from_hex_dec_bin"))]
        val: u8,
    },
}
