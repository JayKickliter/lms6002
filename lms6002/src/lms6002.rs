use algo;
use error::*;
use interface::Interface;
use reg;

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
        self.iface.read(addr).or(Err(Error::IO))
    }

    pub fn write(&self, addr: u8, val: u8) -> Result<()> {
        self.iface.write(addr, val).or(Err(Error::IO))
    }

    /// Reads a single LMS6002 register.
    pub fn read_reg<T: reg::LmsReg>(&self) -> Result<T> {
        let addr = T::addr();
        let reg = self.read(addr)?.into();
        trace!("Read {:?} from 0x{:02x}", reg, addr);
        Ok(reg)
    }

    /// Writes a single LMS6002 register.
    pub fn write_reg<T: reg::LmsReg>(&self, reg: T) -> Result<()> {
        trace!("Writing {:?} to 0x{:02x}", reg, T::addr());
        self.write(T::addr(), reg.into())?;
        Ok(())
    }

    /// Performs the read-modify-write operation `op` on a single LMS6002 register.
    pub fn rmw_reg<T, F>(&self, op: F) -> Result<()>
    where
        T: reg::LmsReg,
        F: FnOnce(&mut T),
    {
        trace!("Performing RMW on LMS register at 0x{:02x}", T::addr());
        let mut r: T = self.read_reg()?;
        op(&mut r);
        self.write_reg(r)?;
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
        info!(
            "{} {:?} path.",
            if enable { "Enabling" } else { "Disabling" },
            path
        );
        match path {
            Path::RX => {
                self.rmw_reg(|r05: &mut reg::Top0x05| r05.set_srxen(enable))?;
                self.rmw_reg(|r09: &mut reg::Top0x09| r09.set_rx_dsm_spi_clk_en(enable))?;
            }
            Path::TX => {
                self.rmw_reg(|r05: &mut reg::Top0x05| r05.set_stxen(enable))?;
                self.rmw_reg(|r09: &mut reg::Top0x09| r09.set_tx_dsm_spi_clk_en(enable))?;
            }
        }
        Ok(())
    }

    /// Tunes specified `path` to `freq`.
    pub fn tune(&self, path: Path, freq: f64) -> Result<()> {
        let params = algo::freq_to_params(self.clk, freq)?;
        info!("Tuning {:?} path to {} using {:?}", path, freq, params);

        use reg::*;

        fn select_vco<M: reg::PllMod, N: Interface>(lms: &LMS6002<N>, _m: M) -> Result<()> {
            info!("Selecting VCO");
            use reg::SelVco::*;
            for vco in &[Vco4, Vco3, Vco2, Vco1] {
                debug!("Trying vco {:?}", vco);

                // Set VCO
                lms.rmw_reg(|r: &mut PllReg<Pll0x05, M>| {
                    r.0.set_selvco((*vco).into());
                })?;

                // Set VCOCAP to 0 and read out VTUNE
                let c00_vtune = {
                    lms.rmw_reg(|r: &mut PllReg<Pll0x09, M>| {
                        r.0.set_vcocap(0);
                    })?;
                    lms.read_reg::<PllReg<Pll0x0A, M>>()?.0.vtune()
                };

                // Set VCOCAP to 63 and read out VTUNE
                let c63_vtune = {
                    lms.rmw_reg(|r: &mut PllReg<Pll0x09, M>| {
                        r.0.set_vcocap(63);
                    })?;
                    lms.read_reg::<PllReg<Pll0x0A, M>>()?.0.vtune()
                };

                if c00_vtune == VTune::High && c63_vtune == VTune::Low {
                    debug!("VCO {:?} locked, selected", vco);
                    return Ok(());
                } else {
                    debug!(
                        "VCO {:?} did not lock. VCO00: {:?}, VCO63: {:?} ",
                        vco, c00_vtune, c63_vtune
                    );
                }
            }
            Err(Error::Range)
        }

        fn select_vcocap<M: reg::PllMod, N: Interface>(lms: &LMS6002<N>, _m: M) -> Result<()> {
            info!("Selecting VCOCAP");
            use reg::VTune::*;

            let set_vcocap = |val: u8| -> Result<()> {
                debug!("Trying VCOCAP {:2}", val);
                lms.rmw_reg(|r: &mut PllReg<Pll0x09, M>| {
                    r.0.set_vcocap(val);
                })?;
                Ok(())
            };
            let read_vtune = || -> Result<VTune> {
                let vtune = lms.read_reg::<PllReg<Pll0x0A, M>>()?.0.vtune();
                Ok(vtune)
            };

            let mut capval_low: Option<u8> = None;
            let mut capval_high: Option<u8> = None;

            // Cycle through all possible VCOCAP values and record VCOCAP values
            // when VTune transitions:
            // - `capval_low`: `High` -> `InRange`
            // - `capval_high`: `InRange` -> `Low`
            for capval in 0..64 {
                set_vcocap(capval)?;
                match (capval_low, capval_high, read_vtune()?) {
                    (None, None, High) => (),
                    t @ (None, None, InRange) => {
                        debug!("VTune transitioned to {:?}", t.2);
                        capval_low = Some(capval);
                    }
                    (Some(_), None, InRange) => continue,
                    t @ (Some(_), None, Low) => {
                        debug!("VTune transitioned to {:?}", t.2);
                        capval_high = Some(capval);
                        break;
                    }
                    t => {
                        error!("Breaking: {:?}", t);
                        break;
                    }
                }
            }

            if let (Some(capval_low), Some(capval_high)) = (capval_low, capval_high) {
                let capval_middle = (capval_low + capval_high) / 2;
                debug!(
                    "Selected VCOCAP {:2} (Low: {:2} High: {:2})",
                    capval_middle, capval_low, capval_high
                );
                Ok(())
            } else {
                error!("Failed to find a suitable VCOCAP value");
                Err(Error::Range)
            }
        }

        fn tune<M: reg::PllMod, N: Interface>(
            lms: &LMS6002<N>,
            m: M,
            params: &algo::TuningParams,
        ) -> Result<()> {
            let algo::TuningParams {
                selvco,
                frange,
                nint,
                nfrac,
            } = *params;

            let mut pll0 = Pll0x00(0);
            pll0.set_nint_8_1((nint >> 1) as u8);
            lms.write_reg(PllReg::new(pll0, m))?;

            let mut pll1 = Pll0x01(0);
            pll1.set_nint_0((nint & 1) as u8);
            pll1.set_nfrac_22_16((nfrac >> 16) as u8);
            lms.write_reg(PllReg::new(pll1, m))?;

            let mut pll2 = Pll0x02(0);
            pll2.set_nfrac_15_8((nfrac >> 8) as u8);
            lms.write_reg(PllReg::new(pll2, m))?;

            let mut pll3 = Pll0x03(0);
            pll3.set_nfrac_7_0(nfrac as u8);
            lms.write_reg(PllReg::new(pll3, m))?;

            lms.rmw_reg(|r: &mut PllReg<Pll0x05, M>| {
                r.0.set_selvco(selvco.into());
                r.0.set_frange(frange.into());
            })?;

            select_vco(lms, m)?;
            select_vcocap(lms, m)?;

            Ok(())
        }

        match path {
            Path::RX => tune(self, reg::RxPll, &params),
            Path::TX => tune(self, reg::TxPll, &params),
        }
    }
}
