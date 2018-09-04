/// The result type returned by `LMS6002` methods.
pub type Result<T> = ::std::result::Result<T, Error>;

/// Error type returned by `LMS6002` methods.
#[derive(Fail, Debug, PartialEq)]
pub enum Error {
    /// An error originating from the underlying (user provided)
    /// interface.
    #[fail(display = "IO")]
    Io,
    /// A parameter is not in range.
    #[fail(display = "Range")]
    Range,
    /// PLL related error.
    #[fail(display = "(PLL) {}", _0)]
    Pll(PllError),
    /// Calibration error.
    #[fail(display = "Failed to calibrate DC offset")]
    Cal,
}

/// An error indicating failure to find suitable PLL parameters.
#[derive(Fail, Debug, PartialEq)]
pub enum PllError {
    #[fail(display = "Failure to find a suitable VCO")]
    VcoSel,
    #[fail(display = "Failure to find a suitable VCOCAP value")]
    VcoCapSel,
}
