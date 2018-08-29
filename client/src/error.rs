use lms6002;
use std::io;

pub type Result = ::std::result::Result<(), Error>;

#[derive(Fail, Debug)]
pub enum Error {
    #[fail(display = "{}", _0)]
    Io(io::Error),
    #[fail(display = "{}", _0)]
    Lms(lms6002::Error),
}

impl From<io::Error> for Error {
    fn from(e: io::Error) -> Error {
        Error::Io(e)
    }
}

impl From<lms6002::Error> for Error {
    fn from(e: lms6002::Error) -> Error {
        Error::Lms(e)
    }
}
