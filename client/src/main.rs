extern crate env_logger;
extern crate failure;
#[macro_use]
extern crate failure_derive;
extern crate lms6002;
extern crate log;
#[cfg(feature = "spi")]
extern crate spidev;
#[macro_use]
extern crate structopt;
mod cmdline;
mod error;
mod interface;
use cmdline::*;
use std::process;
use structopt::StructOpt;

fn reg_cmd(lms: &lms6002::LMS6002<interface::Interface>, cmd: &RegCmd) -> error::Result {
    match *cmd {
        RegCmd::read { addr } => {
            let val = lms.read(addr)?;
            println!("{:#02x}: {:#?}", addr, lms6002::reg::into_debug(addr, val)?);
        }
        RegCmd::write { addr, val } => {
            lms.write(addr, val)?;
        }
    }
    Ok(())
}

fn try_main(opts: Opts) -> error::Result {
    use lms6002::Path;
    let iface = interface::Interface::open(opts.dev)?;
    let lms = lms6002::LMS6002::new(iface, 40_000_000);
    match opts.cmd {
        Cmd::reg(cmd) => reg_cmd(&lms, &cmd)?,
        Cmd::rxpll(TRxCmd::enable) => lms.trx_enable(Path::RX, true)?,
        Cmd::txpll(TRxCmd::enable) => lms.trx_enable(Path::TX, true)?,
        Cmd::rxpll(TRxCmd::disable) => lms.trx_enable(Path::RX, false)?,
        Cmd::txpll(TRxCmd::disable) => lms.trx_enable(Path::TX, false)?,
        Cmd::rxpll(TRxCmd::freq { freq: Some(freq) }) => lms.set_freq(Path::RX, freq)?,
        Cmd::txpll(TRxCmd::freq { freq: Some(freq) }) => lms.set_freq(Path::TX, freq)?,
        Cmd::rxpll(TRxCmd::freq { freq: None }) => {
            println!("{:.1}", lms.freq(Path::RX)?);
        }
        Cmd::txpll(TRxCmd::freq { freq: None }) => {
            println!("{:.1}", lms.freq(Path::TX)?);
        }
    };
    Ok(())
}

fn main() {
    env_logger::init();
    let opts = Opts::from_args();
    match try_main(opts) {
        Ok(_) => process::exit(0),
        Err(e) => {
            eprintln!("ERROR: {}", e);
            process::exit(1);
        }
    }
}
