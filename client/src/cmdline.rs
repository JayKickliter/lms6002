use std::path::PathBuf;

use std::num::ParseIntError;

/// Will attempt to parse an integer from a hex, binary, or decimal
/// string based on the leading two characters:
/// - `0x`: parse as hex
/// - `0b`: binary
/// - other: decimal
trait FromHexDecBin: Sized {
    type Error;
    fn from_hex_dec_bin(&str) -> Result<Self, Self::Error>;
}

macro_rules! impl_from_hex_dec_bin {
    ($T:tt, $E:ty) => {
        impl FromHexDecBin for $T {
            type Error = $E;
            fn from_hex_dec_bin(s: &str) -> Result<$T, Self::Error> {
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

impl_from_hex_dec_bin!(u8, ParseIntError);

#[derive(StructOpt, Debug)]
pub struct Opts {
    #[structopt(short = "l", parse(from_occurrences))]
    // Adds more logging for every `-l[l...]`
    pub log_level: u64,

    #[structopt(parse(from_os_str))]
    pub dev: PathBuf,

    #[structopt(subcommand)]
    pub cmd: Cmd,
}

#[derive(Debug, StructOpt)]
pub enum Cmd {
    /// Direct register manipulation
    #[structopt(name = "reg")]
    Reg {
        #[structopt(parse(try_from_str = "FromHexDecBin::from_hex_dec_bin"))]
        addr: u8,
        #[structopt(
            long = "write",
            name = "val",
            parse(try_from_str = "FromHexDecBin::from_hex_dec_bin")
        )]
        /// Write val
        write: Option<u8>,
    },
}
