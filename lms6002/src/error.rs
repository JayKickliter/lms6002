/// The result type returned by `LMS6002` methods.
pub type Result<T> = ::std::result::Result<T, Error>;

/// Error type returned by `LMS6002` methods.
#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    /// An error originating from the underlying (user provided)
    /// interface.
    Io,
    /// A parameter is not in range.
    Range,
}
