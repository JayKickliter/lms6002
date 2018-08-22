use std::path::PathBuf;

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
        addr: u8,
        #[structopt(long = "write", name = "val")]
        /// Write val
        write: Option<u8>,
    },
}
