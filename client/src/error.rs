use lms6002;
use std::fmt;
use std::io;

pub type Result = ::std::result::Result<(), Error>;

#[derive(Debug)]
pub enum Error {
    Io(io::Error),
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

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Error::Io(_) => write!(f, "IO"),
            Error::Lms(_) => write!(f, "LMS6002"),
        }
    }
}
