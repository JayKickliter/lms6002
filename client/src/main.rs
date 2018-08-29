extern crate env_logger;
extern crate lms6002;
extern crate log;
#[cfg(feature = "spi")]
extern crate spidev;
#[macro_use]
extern crate structopt;

mod cmdline;
mod interface;

use cmdline::*;
use std::io;
use std::process;
use structopt::StructOpt;

type Result = ::std::result::Result<(), Error>;

#[derive(Debug)]
enum Error {
    Io(io::Error),
    Lms(lms6002::Error),
}

impl std::convert::From<io::Error> for Error {
    fn from(e: io::Error) -> Error {
        Error::Io(e)
    }
}

impl std::convert::From<lms6002::Error> for Error {
    fn from(e: lms6002::Error) -> Error {
        Error::Lms(e)
    }
}

fn reg_cmd(lms: &lms6002::LMS6002<interface::Interface>, cmd: &RegCmd) -> Result {
    match *cmd {
        RegCmd::read { addr } => {
            let val = lms.read(addr)?;
            println!(
                "0x{:02x}: {:#?}",
                addr,
                lms6002::reg::into_debug(addr, val)?
            );
        }
        RegCmd::write { addr, val } => {
            lms.write(addr, val)?;
        }
    }
    Ok(())
}

fn try_main(opts: Opts) -> Result {
    use lms6002::Path;
    let iface = interface::Interface::open(opts.dev)?;
    let lms = lms6002::LMS6002::new(iface, 40_000_000);
    let _: () = match opts.cmd {
        Cmd::reg(cmd) => reg_cmd(&lms, &cmd)?,
        Cmd::rx(TRxCmd::enable) => lms.trx_enable(Path::RX, true)?,
        Cmd::tx(TRxCmd::enable) => lms.trx_enable(Path::TX, true)?,
        Cmd::rx(TRxCmd::disable) => lms.trx_enable(Path::RX, false)?,
        Cmd::tx(TRxCmd::disable) => lms.trx_enable(Path::TX, false)?,
        Cmd::rx(TRxCmd::tune { freq }) => lms.set_freq(Path::RX, freq)?,
        Cmd::tx(TRxCmd::tune { freq }) => lms.set_freq(Path::TX, freq)?,
        _ => unimplemented!(),
    };

    Ok(())
}

fn main() {
    env_logger::init();
    let opts = Opts::from_args();
    match try_main(opts) {
        Ok(_) => process::exit(0),
        Err(e) => {
            eprintln!("{:?}", e);
            process::exit(1);
        }
    }
}
