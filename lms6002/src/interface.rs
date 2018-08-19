/// A low level interface for communicating the LMS6002.
pub trait Interface {
    /// Returns the value of the LMS6002's register at address `addr`.
    fn read(&self, addr: u8) -> Result<u8, ()>;

    /// Writes `val` to the LMS6002's register at address `addr`.
    fn write(&self, addr: u8, val: u8) -> Result<(), ()>;
}
