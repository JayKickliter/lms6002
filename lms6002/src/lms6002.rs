use error::Error;
use interface::Interface;
use reg;

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
    pub fn read(&self, addr: u8) -> Result<u8> {
        match self.iface.read(addr) {
            Ok(val) => {
                debug!("Read 0x{:02x} from 0x{:02x}", val, addr);
                Ok(val)
            }
            Err(_) => {
                error!("Failed to read LMS register at 0x{:02x}", addr);
                Err(Error::IO)
            }
        }
    }

    pub fn write(&self, addr: u8, val: u8) -> Result<()> {
        match self.iface.write(addr, val) {
            Ok(_) => {
                debug!("Wrote 0x{:02x} to 0x{:02x}", val, addr);
                Ok(())
            }
            Err(_) => {
                error!("Failed to write to LMS register at 0x{:02x}", addr);
                Err(Error::IO)
            }
        }
    }

    /// Reads a single LMS6002 register.
    pub fn read_reg<T: reg::LmsReg>(&self) -> Result<T> {
        let addr = T::addr();
        let reg = self.read(addr)?.into();
        debug!("Read {:?} from 0x{:02x}", reg, addr);
        Ok(reg)
    }

    /// Writes a single LMS6002 register.
    pub fn write_reg<T: reg::LmsReg>(&self, reg: T) -> Result<()> {
        debug!("Writing {:?} to {}", reg, T::addr());
        self.write(T::addr(), reg.into())?;
        Ok(())
    }

    /// Performs the read-modify-write operation `op` on a single LMS6002 register.
    pub fn rmw_reg<T, F>(&self, op: F) -> Result<()>
    where
        T: reg::LmsReg,
        F: FnOnce(T) -> T,
    {
        debug!("Performing RMW on LMS register at 0x{:02x}", T::addr());
        let r: T = self.read_reg()?;
        let m = op(r);
        self.write_reg(m)?;
        Ok(())
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

    /// Consumes `self` and returns the inner interface.
    pub fn into_inner(self) -> I {
        self.iface
    }

    /// Performs operation `op` on the interface.
    pub fn with_inner<F, R>(&self, op: F) -> R
    where
        F: FnOnce(&I) -> R,
    {
        op(&self.iface)
    }

    /// Performs mutable operation `op` on the interface.
    pub fn with_inner_mut<F, R>(&mut self, op: F) -> R
    where
        F: FnOnce(&mut I) -> R,
    {
        op(&mut self.iface)
    }

    /// [En,Dis]ables the TX or RX path.
    pub fn trx_enable(&self, path: Path, enable: bool) -> Result<()> {
        let endis = if enable { "Enabling" } else { "Disabling" };
        info!("{} {:?} path.", endis, path);
        let mut r05 = self.read_reg::<reg::Top0x05>()?;
        let mut r09 = self.read_reg::<reg::Top0x09>()?;
        match path {
            Path::RX => {
                r05.set_srxen(enable);
                r09.set_rx_dsm_spi_clk_en(enable);
            }
            Path::TX => {
                r05.set_stxen(enable);
                r09.set_tx_dsm_spi_clk_en(enable);
            }
        }
        Ok(())
    }
}
