extern crate env_logger;
extern crate lms6002;
extern crate log;
#[cfg(feature = "spi")]
extern crate spidev;
#[macro_use]
extern crate structopt;
use structopt::StructOpt;

mod cmdline;
mod device;

use cmdline::*;

fn go(opts: Opts) {
    let iface = device::Device::open(opts.dev).unwrap();
    let lms = lms6002::LMS6002::new(iface, 40_000_000);
    match opts.cmd {
        Cmd::Reg { addr, write: None } => {
            let val = lms.read(addr).unwrap();
            println!(
                "0x{:02x}: {:#?}",
                addr,
                lms6002::reg::into_debug(addr, val).unwrap()
            );
        }
        Cmd::Reg {
            addr,
            write: Some(val),
        } => {
            lms.write(addr, val).unwrap();
        }
    }
}

fn main() {
    env_logger::init();
    let opts = Opts::from_args();
    go(opts);
}
