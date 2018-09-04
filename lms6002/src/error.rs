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
    #[fail(display = "(CAL) {}", _0)]
    Cal(CalError),
}

/// An error indicating failure to find suitable PLL parameters.
#[derive(Fail, Debug, PartialEq)]
pub enum PllError {
    #[fail(display = "Failure to find a suitable VCO")]
    VcoSel,
    #[fail(display = "Failure to find a suitable VCOCAP value")]
    VcoCapSel,
}

/// An error indicating failure in DC offset calibration.
#[derive(Fail, Debug, PartialEq)]
pub enum CalError {
    /// Reached TRY_CNT limit without finishing DC CAL.
    #[fail(
        display = "Failure to finish DC CAL of module at addr {:#02} after {} tries",
        _0,
        _1
    )]
    TryCntLimReached(u8, usize),
}
