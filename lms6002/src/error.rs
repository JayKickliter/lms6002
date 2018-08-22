/// Error type returned by `LMS6002` methods.
#[derive(Debug)]
pub enum Error {
    /// An error originating from the underlying (user provided)
    /// interface.
    IO,
}
