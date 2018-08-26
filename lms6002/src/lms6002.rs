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

/// Low-level API.
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
}

impl<I: Interface> LMS6002<I> {
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

    /// [En,Dis]ables the TX or RX path.
    pub fn trx_enable(&self, path: Path, enable: bool) -> Result<()> {
        let endis = if enable { "Enabling" } else { "Disabling" };
        info!("{} {:?} path.", endis, path);
        let mut r05: reg::Top0x05 = self.read_reg()?;
        let mut r09: reg::Top0x09 = self.read_reg()?;
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
        self.write_reg(r05)?;
        self.write_reg(r09)?;
        Ok(())
    }
}

const FREQSEL_LUT: [(u32, u32, u8); 16] = [
    (232_500_000, 285_625_000, 0b100111),
    (285_625_000, 336_875_000, 0b101111),
    (336_875_000, 405_000_000, 0b110111),
    (405_000_000, 465_000_000, 0b111111),
    (465_000_000, 571_250_000, 0b100110),
    (571_250_000, 673_750_000, 0b101110),
    (673_750_000, 810_000_000, 0b110110),
    (810_000_000, 930_000_000, 0b111110),
    (930_000_000, 1_142_500_000, 0b100101),
    (1_142_500_000, 1_347_500_000, 0b101101),
    (1_347_500_000, 1_620_000_000, 0b110101),
    (1_620_000_000, 1_860_000_000, 0b111101),
    (1_860_000_000, 2_285_000_000, 0b100100),
    (2_285_000_000, 2_695_000_000, 0b101100),
    (2_695_000_000, 3_240_000_000, 0b110100),
    (3_240_000_000, 3_720_000_000, 0b111100),
];

fn freqsel(freq: u32) -> Option<u8> {
    FREQSEL_LUT
        .iter()
        .find(|&&(l, h, _)| l < freq && freq < h)
        .map(|&(_, _, val)| val)
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_freqsel() {
        use super::{freqsel, FREQSEL_LUT};
        for &(l, h, val) in &FREQSEL_LUT {
            let freq = (h - l) / 2 + l;
            assert_eq!(freqsel(freq), Some(val));
            let freq = l - 1;
            assert!(freqsel(freq) != Some(val));
            let freq = h + 1;
            assert!(freqsel(freq) != Some(val));
        }
    }
}
