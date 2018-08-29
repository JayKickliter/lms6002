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
use structopt::StructOpt;

fn reg_cmd(lms: &lms6002::LMS6002<interface::Interface>, cmd: &RegCmd) {
    match *cmd {
        RegCmd::read { addr } => {
            let val = lms.read(addr).unwrap();
            println!(
                "0x{:02x}: {:#?}",
                addr,
                lms6002::reg::into_debug(addr, val).unwrap()
            );
        }
        RegCmd::write { addr, val } => {
            lms.write(addr, val).unwrap();
        }
    }
}

fn go(opts: Opts) {
    use lms6002::Path;
    let iface = interface::Interface::open(opts.dev).unwrap();
    let lms = lms6002::LMS6002::new(iface, 40_000_000);
    match opts.cmd {
        Cmd::reg(cmd) => reg_cmd(&lms, &cmd),
        Cmd::rx(TRxCmd::enable) => lms.trx_enable(Path::RX, true).unwrap(),
        Cmd::tx(TRxCmd::enable) => lms.trx_enable(Path::TX, true).unwrap(),
        Cmd::rx(TRxCmd::disable) => lms.trx_enable(Path::RX, false).unwrap(),
        Cmd::tx(TRxCmd::disable) => lms.trx_enable(Path::TX, false).unwrap(),
        Cmd::rx(TRxCmd::tune { freq }) => lms.set_freq(Path::RX, freq).unwrap(),
        Cmd::tx(TRxCmd::tune { freq }) => lms.set_freq(Path::TX, freq).unwrap(),

        cmd => panic!("Unhandled command: {:?}", cmd),
    }
}

fn main() {
    env_logger::init();
    let opts = Opts::from_args();
    go(opts);
}
