use error::Error;
use interface::Interface;

/// The result type returned by `LMS6002` methods.
pub type Result<T> = ::std::result::Result<T, Error>;

/// A high-level interface for configuring and controlling an LMS6002.
#[derive(Debug)]
pub struct LMS6002<I> {
    /// The low-level register read/write interface provided by the
    /// user.
    #[doc(hidden)]
    iface: I,
    /// Hardware-specific reference clock speed for user's LMS6002.
    #[doc(hidden)]
    clk: u32,
}

/// Represents either the transmit or receive paths on the `LMS6002`.
///
/// Many `LMS6002` methods are common between `RX` and `TX`. For
/// those, we specify which one we want to operate on by passing in
/// this enum.
#[derive(Debug, Clone, Copy)]
pub enum Path {
    /// Receive path.
    RX,
    /// Transmit path.
    TX,
}

impl<I: Interface> LMS6002<I> {
    fn read(&self, addr: u8) -> Result<u8> {
        match self.iface.read(addr) {
            Ok(val) => Ok(val),
            Err(_) => Err(Error::IO),
        }
    }

    fn write(&self, addr: u8, val: u8) -> Result<()> {
        match self.iface.write(addr, val) {
            Ok(_) => Ok(()),
            Err(_) => Err(Error::IO),
        }
    }

    /// Returns a new `LMS6002`.
    ///
    /// ## Parameters
    ///
    /// * `iface`: a user-defined type that implements `Interface`.
    /// * `clk`: reference clock speed for your chip.
    ///          This is application-specific, thus a parameter.
    pub fn new(iface: I, clk: u32) -> Self {
        Self { iface, clk }
    }
}
