/// The result type returned by `LMS6002` methods.
pub type Result<T> = ::std::result::Result<T, Error>;

/// Error type returned by `LMS6002` methods.
#[derive(Debug, PartialEq)]
pub enum Error {
    /// An error originating from the underlying (user provided)
    /// interface.
    IO,
    /// A parameter is not in range.
    Range,
}
