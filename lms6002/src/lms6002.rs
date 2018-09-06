use algo;
use error::*;
use interface::Interface;
use reg;
use std::thread;
use std::time;

struct RegStash<'a, R: reg::LmsReg, I: 'a + Interface> {
    reg: R,
    lms: &'a LMS6002<I>,
}

impl<'a, R: reg::LmsReg, I: Interface> RegStash<'a, R, I> {
    pub fn new(lms: &'a LMS6002<I>) -> Result<Self> {
        let reg = lms.read_reg()?;
        debug!("Backed up {:?}", reg);
        Ok(RegStash { reg: reg, lms })
    }

    pub fn restore(&self) {
        match self.lms.write_reg(self.reg) {
            Ok(()) => debug!("Restoring {:?}", self.reg),
            Err(e) => error!("Could not restore register: {}", e),
        }
    }
}

impl<'a, R: reg::LmsReg, I: Interface> Drop for RegStash<'a, R, I> {
    fn drop(&mut self) {
        self.restore();
    }
}

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
        self.iface.read(addr).or(Err(Error::Io))
    }

    pub fn write(&self, addr: u8, val: u8) -> Result<()> {
        self.iface.write(addr, val).or(Err(Error::Io))
    }

    /// Reads a single LMS6002 register.
    pub fn read_reg<T: reg::LmsReg>(&self) -> Result<T> {
        let addr = T::addr();
        let reg = self.read(addr)?.into();
        trace!("Read {:?} from {:#02x}", reg, addr);
        Ok(reg)
    }

    /// Writes a single LMS6002 register.
    pub fn write_reg<T: reg::LmsReg>(&self, reg: T) -> Result<()> {
        trace!("Writing {:?} to {:#02x}", reg, T::addr());
        self.write(T::addr(), reg.into())?;
        Ok(())
    }

    /// Performs the read-modify-write operation `op` on a single LMS6002 register.
    pub fn rmw_reg<T, F>(&self, op: F) -> Result<()>
    where
        T: reg::LmsReg,
        F: FnOnce(&mut T),
    {
        trace!("Performing RMW on LMS register at {:#02x}", T::addr());
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

    fn stash<R>(&self) -> Result<RegStash<R, I>>
    where
        R: reg::LmsReg,
    {
        RegStash::new(self)
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

    fn tuning_params(&self, path: Path) -> Result<algo::TuningParams> {
        fn tuning_params<M: reg::PllMod, N: Interface>(
            lms: &LMS6002<N>,
            _m: M,
        ) -> Result<algo::TuningParams> {
            use reg::*;

            let pll0 = lms.read_reg::<PllReg<Pll0x00, M>>()?.0;
            let pll1 = lms.read_reg::<PllReg<Pll0x01, M>>()?.0;
            let pll2 = lms.read_reg::<PllReg<Pll0x02, M>>()?.0;
            let pll3 = lms.read_reg::<PllReg<Pll0x03, M>>()?.0;

            let pll5 = lms.read_reg::<PllReg<Pll0x05, M>>()?.0;
            Ok(algo::TuningParams {
                selvco: pll5.selvco(),
                frange: pll5.frange(),
                nint: (u16::from(pll0.nint_8_1()) << 1) | u16::from(pll1.nint_0()),
                nfrac: ((u32::from(pll1.nfrac_22_16())) << 16)
                    | ((u32::from(pll2.nfrac_15_8())) << 8)
                    | u32::from(pll3.nfrac_7_0()),
            })
        }
        let params = match path {
            Path::RX => tuning_params(self, reg::RxPll)?,
            Path::TX => tuning_params(self, reg::TxPll)?,
        };
        debug!("Read {:?} tuning params: {:?}", path, params);
        Ok(params)
    }

    /// Returns `path`'s current frequency.
    pub fn freq(&self, path: Path) -> Result<f64> {
        let params = self.tuning_params(path)?;
        Ok(algo::params_to_freq(self.clk, &params))
    }

    /// Tunes specified `path` to `freq`.
    pub fn set_freq(&self, path: Path, freq: f64) -> Result<()> {
        let params = algo::params_from_freq(self.clk, freq)?;
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
            Err(Error::Pll(PllError::VcoSel))
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
                        capval_high = Some(capval - 1);
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
                    "Selected VCOCAP: (Low: {:2} < Selected: {:2} < High: {:2})",
                    capval_low, capval_middle, capval_high
                );
                Ok(())
            } else {
                error!("Failed to find a suitable VCOCAP value");
                Err(Error::Pll(PllError::VcoCapSel))
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

            Ok(())
        }

        match path {
            Path::RX => {
                let pll_mod = reg::RxPll;
                tune(self, pll_mod, &params)?;
                select_vco(self, pll_mod)?;
                select_vcocap(self, pll_mod)?;
            }
            Path::TX => {
                let pll_mod = reg::TxPll;
                tune(self, pll_mod, &params)?;
                select_vco(self, pll_mod)?;
                select_vcocap(self, pll_mod)?;
            }
        }

        Ok(())
    }

    /// Returns `path`'s low-pass filter bandwidth (in Hz).
    pub fn lpf_bw(&self, path: Path) -> Result<f64> {
        let bwc = match path {
            Path::RX => self.read_reg::<reg::RxLpfDacAdc0x54>()?.bwc_lpf(),
            Path::TX => self.read_reg::<reg::TxLpf0x34>()?.bwc_lpf(),
        };
        let bw = algo::bw_from_bwc(bwc)?;
        info!("Read {} for {:?} LPF bandwidth", bw, path);
        Ok(bw)
    }

    /// Sets specified `path`'s low-pass filter to `bw`.
    pub fn set_lpf_bw(&self, path: Path, bw: f64) -> Result<()> {
        let bwc = algo::bwc_from_bw(bw)?;
        info!("Setting {:?} LPF to {:#04b} ({} Hz)", path, bwc, bw);
        match path {
            Path::RX => self.rmw_reg(|r: &mut reg::RxLpfDacAdc0x54| r.set_bwc_lpf(bwc)),
            Path::TX => self.rmw_reg(|r: &mut reg::TxLpf0x34| r.set_bwc_lpf(bwc)),
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum LpfDcCalChan {
    I = 0,
    Q = 1,
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum RxVga2DcCalChan {
    DcRef = 0b000,
    FirstStageI = 0b001,
    FirstStageQ = 0b010,
    SecondStageI = 0b011,
    SecondStageQ = 0b100,
}

#[derive(Debug, Clone, Copy)]
pub enum DcCalMod {
    LpfTuning,
    RxLpf(Option<LpfDcCalChan>),
    TxLpf(Option<LpfDcCalChan>),
    RxVga2(Option<RxVga2DcCalChan>),
}

impl DcCalMod {
    fn set_clk(self, reg: reg::Top0x09, en: bool) -> reg::Top0x09 {
        let mut reg = reg;
        match self {
            DcCalMod::LpfTuning => reg.set_lpf_cal_clk_en(en),
            DcCalMod::RxLpf(_) => reg.set_rx_lpf_dccal_clk_en(en),
            DcCalMod::TxLpf(_) => reg.set_tx_lpf_spi_dccal_clk_en(en),
            DcCalMod::RxVga2(_) => reg.set_rx_vga2_dccal_en(en),
        };
        reg
    }
}

impl<I: Interface> LMS6002<I> {
    /// Genereral DC calibration procedure.
    ///
    /// This function is backbone of all other module-specific DC
    /// calibration subroutines. It is outlined in "LMS6002
    /// Programming and Calibration",
    /// [Sec. 4.1](https://wiki.myriadrf.org/LimeMicro:LMS6002D_Programming_and_Calibration#General_DC_Calibration_Procedure)
    pub fn general_dc_cal<R, S, C>(&self, addr: u8) -> Result<u8>
    where
        C: reg::DcCalControlReg + reg::LmsReg,
        S: reg::DcCalStatusReg + reg::LmsReg,
        R: reg::DcRegValReg + reg::LmsReg,
    {
        fn inner<C, S, N>(lms: &LMS6002<N>, addr: u8) -> Result<()>
        where
            C: reg::DcCalControlReg + reg::LmsReg,
            S: reg::DcCalStatusReg + reg::LmsReg,
            N: Interface,
        {
            // Max number tries before giving up on cal. Arbitrarily
            // chosen value.
            const TRYCNT: usize = 10;
            let mut ctrl: C = lms.read_reg()?;

            // DC_ADDR := ADDR
            ctrl.set_addr(addr);

            // DC_START_CLBR := 1
            ctrl.set_start_clbr(true);
            lms.write_reg(ctrl)?;
            ctrl = lms.read_reg()?;

            // DC_START_CLBR := 0
            ctrl.set_start_clbr(false);
            lms.write_reg(ctrl)?;

            info!("Performing DC offset cal for module at {:#02x}", addr);
            for i in 0..TRYCNT {
                debug!("DC CAL {:#02x} attempt {}/{}", addr, i + 1, TRYCNT);
                // Wait fo 6.4 uS
                thread::sleep(time::Duration::new(0, 6_400));

                let sts: S = lms.read_reg()?;

                // Check DC_CLBR_DONE
                if !sts.clbr_done() {
                    continue;
                }

                // Check DC_LOCK
                if sts.lock() {
                    debug!("DC CAl {:#02x}, got DC_LOCK", addr);
                    break;
                }
                // Keep going
            }
            Ok(())
        }

        // First try straight calibration routine. As long as
        // `DC_REGVAL != 31`, we can trust that worked and can return.
        self.rmw_reg(|reg: &mut R| reg.set_regval(31))?;
        inner::<C, S, I>(self, addr)?;
        let dc_regval = self.read_reg::<R>()?.regval();
        if dc_regval != 31 {
            debug!("First calibration attempt succeeded");
            return Ok(dc_regval);
        }
        warn!("First calibration attempt failed (`DC_REGVAL = 31`)");

        // The first attempt didn't converge. This time we'll set
        // DC_REGVAL to 0 and run the calibration routine again. If we
        // succeed, success is indicated by a value other than 0 when
        // we're done.
        debug!("Trying calibration a second time with `DC_REGVAL = 0`");
        self.rmw_reg(|reg: &mut R| reg.set_regval(0))?;
        inner::<C, S, I>(self, addr)?;
        let dc_regval = self.read_reg::<R>()?.regval();
        if dc_regval != 0 {
            return Ok(dc_regval);
        }
        error!("Second calibration attempt failed: `DC_REGVAL = 0`");

        Err(Error::Cal)
    }

    pub fn dc_cal(&self, module: DcCalMod) -> Result<()> {
        use reg::*;
        info!("Performing DC offset calibration for {:?}", module);
        // Stash clk enable register and restore it after calibrating
        // specified module
        let top09 = self.stash::<Top0x09>()?;

        // Enable relevant clock for this module.
        debug!("Temporarily enabling clk for {:?}", module);
        self.write_reg(module.set_clk(top09.reg, true))?;

        fn inner<N: Interface>(lms: &LMS6002<N>, module: DcCalMod) -> Result<()> {
            match module {
                DcCalMod::LpfTuning => {
                    let dccal = lms.general_dc_cal::<Top0x00, Top0x01, Top0x03>(0)?;
                    lms.rmw_reg(|reg: &mut TxLpf0x35| reg.set_dco_daccal(dccal))?;
                    lms.rmw_reg(|reg: &mut RxLpfDacAdc0x55| reg.set_dco_daccal(dccal))?;
                }
                DcCalMod::RxLpf(Some(chan)) => {
                    lms.general_dc_cal::<RxLpfDacAdc0x50, RxLpfDacAdc0x51, RxLpfDacAdc0x53>(
                        chan as u8,
                    )?;
                }
                DcCalMod::RxLpf(None) => {
                    lms.general_dc_cal::<RxLpfDacAdc0x50, RxLpfDacAdc0x51, RxLpfDacAdc0x53>(
                        LpfDcCalChan::I as u8,
                    )?;
                    lms.general_dc_cal::<RxLpfDacAdc0x50, RxLpfDacAdc0x51, RxLpfDacAdc0x53>(
                        LpfDcCalChan::Q as u8,
                    )?;
                }
                DcCalMod::TxLpf(Some(chan)) => {
                    lms.general_dc_cal::<TxLpf0x30, TxLpf0x31, TxLpf0x33>(chan as u8)?;
                }
                DcCalMod::TxLpf(None) => {
                    lms.general_dc_cal::<TxLpf0x30, TxLpf0x31, TxLpf0x33>(LpfDcCalChan::I as u8)?;
                    lms.general_dc_cal::<TxLpf0x30, TxLpf0x31, TxLpf0x33>(LpfDcCalChan::Q as u8)?;
                }
                DcCalMod::RxVga2(Some(chan)) => {
                    // Stash current gain and temporarily set it to 30 dB
                    let _reg = lms.stash::<RxVga0x65>()?;
                    lms.set_rxvga2_gain(30)?;

                    lms.general_dc_cal::<RxVga0x60, RxVga0x61, RxVga0x63>(chan as u8)?;
                }
                DcCalMod::RxVga2(None) => {
                    // Stash current gain and temporarily set it to 30 dB
                    let _reg = lms.stash::<RxVga0x65>()?;
                    lms.set_rxvga2_gain(30)?;

                    lms.general_dc_cal::<RxVga0x60, RxVga0x61, RxVga0x63>(
                        RxVga2DcCalChan::DcRef as u8,
                    )?;
                    lms.general_dc_cal::<RxVga0x60, RxVga0x61, RxVga0x63>(
                        RxVga2DcCalChan::FirstStageI as u8,
                    )?;
                    lms.general_dc_cal::<RxVga0x60, RxVga0x61, RxVga0x63>(
                        RxVga2DcCalChan::FirstStageQ as u8,
                    )?;
                    lms.general_dc_cal::<RxVga0x60, RxVga0x61, RxVga0x63>(
                        RxVga2DcCalChan::SecondStageI as u8,
                    )?;
                    lms.general_dc_cal::<RxVga0x60, RxVga0x61, RxVga0x63>(
                        RxVga2DcCalChan::SecondStageQ as u8,
                    )?;
                }
            }
            Ok(())
        }

        inner(self, module)
    }
}

impl<I: Interface> LMS6002<I> {
    /// Returns RXVGA2's gain, in dB.
    pub fn rxvga2_gain(&self) -> Result<u32> {
        algo::rxga2_gain_from_field(self.read_reg::<reg::RxVga0x65>()?.vga2gain())
    }

    /// Sets RXVGA2's gain in dB.
    ///
    /// **NOTE**: `gain` must be a multiple of 3 and <= 60; errors
    /// otherwise.
    pub fn set_rxvga2_gain(&self, gain: u32) -> Result<()> {
        let field = algo::rxga2_gain_to_field(gain)?;
        self.rmw_reg(|reg: &mut reg::RxVga0x65| {
            reg.set_vga2gain(field);
        })?;
        Ok(())
    }
}
