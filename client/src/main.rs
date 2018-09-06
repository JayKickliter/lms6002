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
extern crate bitfield;
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
        RegCmd::write {
            addr,
            range: None,
            val,
        } => {
            lms.write(addr, val)?;
        }
        RegCmd::write {
            addr,
            range: Some(BitRange(h, l)),
            val,
        } => {
            let mut regval = lms.read(addr)?;
            println!(
                "{:#02x}: -> {:#?}",
                addr,
                lms6002::reg::into_debug(addr, regval)?
            );
            ::bitfield::BitRange::set_bit_range(&mut regval, h, l, val);
            lms.write(addr, regval)?;
            println!(
                "{:#02x}: <- {:#?}",
                addr,
                lms6002::reg::into_debug(addr, regval)?
            );
        }
    }
    Ok(())
}

fn rxvga2_cmd(lms: &lms6002::LMS6002<interface::Interface>, cmd: &RxVga2Cmd) -> error::Result {
    match *cmd {
        RxVga2Cmd::gain { set: None } => println!("{}", lms.rxvga2_gain()?),
        RxVga2Cmd::gain { set: Some(gain) } => {
            lms.set_rxvga2_gain(gain)?;
        }
    }
    Ok(())
}

fn try_main(opts: Opts) -> error::Result {
    use lms6002::{DcCalMod, Path};
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
        Cmd::rxlpf(LpfCmd { freq: Some(freq) }) => lms.set_lpf_bw(Path::RX, freq)?,
        Cmd::txlpf(LpfCmd { freq: Some(freq) }) => lms.set_lpf_bw(Path::TX, freq)?,
        Cmd::rxlpf(LpfCmd { freq: None }) => println!("{:}", lms.lpf_bw(Path::RX)?),
        Cmd::txlpf(LpfCmd { freq: None }) => println!("{:}", lms.lpf_bw(Path::TX)?),
        Cmd::cal(module) => match module {
            CalCmd::lpftuning => lms.dc_cal(DcCalMod::LpfTuning)?,
            CalCmd::rxlpf => lms.dc_cal(DcCalMod::RxLpf(None))?,
            CalCmd::txlpf => lms.dc_cal(DcCalMod::TxLpf(None))?,
            CalCmd::rxvga2 => lms.dc_cal(DcCalMod::RxVga2(None))?,
        },
        Cmd::rxvga2(cmd) => rxvga2_cmd(&lms, &cmd)?,
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
