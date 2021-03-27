//! # Reset & Clock Unit

use core::cmp;

use crate::pac::{
    fmc::{ws::WSCNT_A, WS},
    rcu::{
        self,
        cfg0::{PLLSEL_A, SCS_A, USBDPSC_A},
    },
    RCU,
};
use cast::u32;

//use crate::flash::ACR;
use crate::time::Hertz;

//use crate::backup_domain::BackupDomain;

/// Extension trait that constrains the `RCU` peripheral
pub trait RcuExt {
    /// Constrains the `RCU` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcu;
}

impl RcuExt for RCU {
    fn constrain(self) -> Rcu {
        Rcu {
            ahb: AHB { _0: () },
            apb1: APB1 { _0: () },
            apb2: APB2 { _0: () },
            cfgr: CFGR {
                hse: None,
                hclk: None,
                pclk1: None,
                pclk2: None,
                sysclk: None,
                adcclk: None,
            },
            bkp: BKP { _0: () },
        }
    }
}

/// Constrained RCU peripheral
///
/// Aquired by calling the [constrain](../trait.RcuExt.html#tymethod.constrain) method
/// on the Rcu struct from the `PAC`
///
/// ```rust
/// let dp = pac::Peripherals::take().unwrap();
/// let mut rcu = dp.RCU.constrain();
/// ```
pub struct Rcu {
    /// AMBA High-performance Bus (AHB) registers
    pub ahb: AHB,
    /// Advanced Peripheral Bus 1 (APB1) registers
    pub apb1: APB1,
    /// Advanced Peripheral Bus 2 (APB2) registers
    pub apb2: APB2,
    pub cfgr: CFGR,
    pub bkp: BKP,
}

/// AMBA High-performance Bus (AHB) registers
///
/// Aquired through the `Rcu` registers:
///
/// ```rust
/// let dp = pac::Peripherals::take().unwrap();
/// let mut rcu = dp.RCU.constrain();
/// function_that_uses_ahb(&mut rcu.ahb)
/// ```
pub struct AHB {
    _0: (),
}

impl AHB {
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn enr(&mut self) -> &rcu::AHBEN {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCU::ptr()).ahben }
    }
}

/// Advanced Peripheral Bus 1 (APB1) registers
///
/// Aquired through the `Rcu` registers:
///
/// ```rust
/// let dp = pac::Peripherals::take().unwrap();
/// let mut rcu = dp.RCU.constrain();
/// function_that_uses_apb1(&mut rcu.apb1)
/// ```
pub struct APB1 {
    _0: (),
}

impl APB1 {
    pub(crate) fn enr(&mut self) -> &rcu::APB1EN {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCU::ptr()).apb1en }
    }

    pub(crate) fn rstr(&mut self) -> &rcu::APB1RST {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCU::ptr()).apb1rst }
    }
}

impl APB1 {
    /// Set power interface clock (PWREN) bit in RCU_APB1ENR
    pub fn set_pwren(&mut self) {
        self.enr().modify(|_r, w| w.pmuen().enabled())
    }
}

/// Advanced Peripheral Bus 2 (APB2) registers
///
/// Aquired through the `Rcu` registers:
///
/// ```rust
/// let dp = pac::Peripherals::take().unwrap();
/// let mut rcu = dp.RCU.constrain();
/// function_that_uses_apb2(&mut rcu.apb2);
/// ```
pub struct APB2 {
    _0: (),
}

impl APB2 {
    pub(crate) fn enr(&mut self) -> &rcu::APB2EN {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCU::ptr()).apb2en }
    }

    pub(crate) fn rstr(&mut self) -> &rcu::APB2RST {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCU::ptr()).apb2rst }
    }
}

const HSI: u32 = 8_000_000; // Hz

/// Clock configuration register (CFGR)
///
/// Used to configure the frequencies of the clocks present in the processor.
///
/// After setting all frequencies, call the [freeze](#method.freeze) function to
/// apply the configuration.
///
/// **NOTE**: Currently, it is not guaranteed that the exact frequencies selected will be
/// used, only frequencies close to it.
pub struct CFGR {
    hse: Option<u32>,
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
    adcclk: Option<u32>,
}

impl CFGR {
    /// Uses HSE (external oscillator) instead of HSI (internal RC oscillator) as the clock source.
    /// Will result in a hang if an external oscillator is not connected or it fails to start.
    /// The frequency specified must be the frequency of the external oscillator
    pub fn use_hse<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hse = Some(freq.into().0);
        self
    }

    /// Sets the desired frequency for the HCLK clock
    pub fn hclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hclk = Some(freq.into().0);
        self
    }

    /// Sets the desired frequency for the PCKL1 clock
    pub fn pclk1<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk1 = Some(freq.into().0);
        self
    }

    /// Sets the desired frequency for the PCLK2 clock
    pub fn pclk2<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk2 = Some(freq.into().0);
        self
    }

    /// Sets the desired frequency for the SYSCLK clock
    pub fn sysclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.sysclk = Some(freq.into().0);
        self
    }

    /// Sets the desired frequency for the ADCCLK clock
    pub fn adcclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.adcclk = Some(freq.into().0);
        self
    }

    /// Applies the clock configuration and returns a `Clocks` struct that signifies that the
    /// clocks are frozen, and contains the frequencies used. After this function is called,
    /// the clocks can not change
    ///
    /// Usage:
    ///
    /// ```rust
    /// let dp = pac::Peripherals::take().unwrap();
    /// let mut flash = dp.FLASH.constrain();
    /// let mut rcu = dp.RCU.constrain();
    /// let clocks = rcu.cfgr.freeze(&mut flash.acr);
    /// ```

    pub fn freeze(self, ws: &WS) -> Clocks {
        let pllsrculk = self.hse.unwrap_or(HSI / 2);

        let pllmul = self.sysclk.unwrap_or(pllsrculk) / pllsrculk;

        let (pllmul_bits, sysclk) = if pllmul == 1 {
            (None, self.hse.unwrap_or(HSI))
        } else {
            let pllmul = cmp::min(cmp::max(pllmul, 1), 16);

            (Some(pllmul as u8 - 2), pllsrculk * pllmul)
        };

        assert!(sysclk <= 72_000_000);

        let hpre_bits = self
            .hclk
            .map(|hclk| match sysclk / hclk {
                0 => unreachable!(),
                1 => 0b0111,
                2 => 0b1000,
                3..=5 => 0b1001,
                6..=11 => 0b1010,
                12..=39 => 0b1011,
                40..=95 => 0b1100,
                96..=191 => 0b1101,
                192..=383 => 0b1110,
                _ => 0b1111,
            })
            .unwrap_or(0b0111);

        let hclk = if hpre_bits >= 0b1100 {
            sysclk / (1 << (hpre_bits - 0b0110))
        } else {
            sysclk / (1 << (hpre_bits - 0b0111))
        };

        assert!(hclk <= 72_000_000);

        let ppre1_bits = self
            .pclk1
            .map(|pclk1| match hclk / pclk1 {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3..=5 => 0b101,
                6..=11 => 0b110,
                _ => 0b111,
            })
            .unwrap_or(0b011);

        let ppre1 = 1 << (ppre1_bits - 0b011);
        let pclk1 = hclk / u32(ppre1);

        assert!(pclk1 <= 36_000_000);

        let ppre2_bits = self
            .pclk2
            .map(|pclk2| match hclk / pclk2 {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3..=5 => 0b101,
                6..=11 => 0b110,
                _ => 0b111,
            })
            .unwrap_or(0b011);

        let ppre2 = 1 << (ppre2_bits - 0b011);
        let pclk2 = hclk / u32(ppre2);

        assert!(pclk2 <= 72_000_000);

        // adjust flash wait states
        ws.write(|w| {
            w.wscnt().variant(if sysclk <= 24_000_000 {
                WSCNT_A::WS0
            } else if sysclk <= 48_000_000 {
                WSCNT_A::WS1
            } else {
                WSCNT_A::WS2
            })
        });

        // the USB clock is only valid if an external crystal is used, the PLL is enabled, and the
        // PLL output frequency is a supported one.
        // usbpre == false: divide clock by 1.5, otherwise no division
        let (usbpre, usbclk_valid) = match (self.hse, pllmul_bits, sysclk) {
            (Some(_), Some(_), 72_000_000) => (USBDPSC_A::DIV1_5, true),
            (Some(_), Some(_), 48_000_000) => (USBDPSC_A::DIV1, true),
            _ => (USBDPSC_A::DIV1_5, false),
        };

        let apre_bits: u8 = self
            .adcclk
            .map(|adcclk| match pclk2 / adcclk {
                0..=2 => 0b00,
                3..=4 => 0b01,
                5..=7 => 0b10,
                _ => 0b11,
            })
            .unwrap_or(0b11);

        let apre = (apre_bits + 1) << 1;
        let adcclk = pclk2 / u32(apre);

        assert!(adcclk <= 14_000_000);

        let rcu = unsafe { &*RCU::ptr() };

        if self.hse.is_some() {
            // enable HSE and wait for it to be ready

            rcu.ctl0.modify(|_, w| w.hxtalen().on());

            while rcu.ctl0.read().hxtalstb().is_not_ready() {}
        }

        if let Some(pllmul_bits) = pllmul_bits {
            // enable PLL and wait for it to be ready

            rcu.cfg0.modify(|_, w| {
                w.pllmf()
                    .bits(pllmul_bits)
                    .pllsel()
                    .variant(if self.hse.is_some() {
                        PLLSEL_A::HXTAL
                    } else {
                        PLLSEL_A::IRC8M_2
                    })
            });

            rcu.ctl0.modify(|_, w| w.pllen().on());

            while rcu.ctl0.read().pllstb().is_not_ready() {}
        }

        // set prescalers and clock source
        #[cfg(feature = "gd32f130")]
        rcu.cfg0.modify(|_, w| unsafe {
            w.adcpsc().bits(apre_bits);
            w.apb2psc()
                .bits(ppre2_bits)
                .apb1psc()
                .bits(ppre1_bits)
                .ahbpsc()
                .bits(hpre_bits)
                .usbdpsc()
                .variant(usbpre)
                .scs()
                .variant(if pllmul_bits.is_some() {
                    SCS_A::PLL
                } else if self.hse.is_some() {
                    SCS_A::HXTAL
                } else {
                    SCS_A::IRC8M
                })
        });

        Clocks {
            hclk: Hertz(hclk),
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            ppre1,
            ppre2,
            sysclk: Hertz(sysclk),
            adcclk: Hertz(adcclk),
            usbclk_valid,
        }
    }
}

pub struct BKP {
    _0: (),
}

/*impl BKP {
    /// Enables write access to the registers in the backup domain
    pub fn constrain(self, bkp: crate::pac::BKP, apb1: &mut APB1, pwr: &mut PWR) -> BackupDomain {
        // Enable the backup interface by setting PWREN and BKPEN
        apb1.enr()
            .modify(|_r, w| w.bkpen().set_bit().pwren().set_bit());

        // Enable access to the backup registers
        pwr.cr.modify(|_r, w| w.dbp().set_bit());

        BackupDomain { _regs: bkp }
    }
}*/

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
///
/// To acquire it, use the freeze function on the `rcu.cfgr` register. If desired, you can adjust
/// the frequencies using the methods on [cfgr](struct.CFGR.html) before calling freeze.
///
/// ```rust
/// let dp = pac::Peripherals::take().unwrap();
/// let mut rcu = dp.RCU.constrain();
/// let mut flash = dp.FLASH.constrain();
///
/// let clocks = rcu.cfgr.freeze(&mut flash.acr);
/// ```
#[derive(Clone, Copy)]
pub struct Clocks {
    hclk: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    ppre1: u8,
    ppre2: u8,
    sysclk: Hertz,
    adcclk: Hertz,
    usbclk_valid: bool,
}

impl Clocks {
    /// Returns the frequency of the AHB
    pub fn hclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns the frequency of the APB1
    pub fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of the APB2
    pub fn pclk2(&self) -> Hertz {
        self.pclk2
    }

    /// Returns the frequency of the APB1 Timers
    pub fn pclk1_tim(&self) -> Hertz {
        Hertz(self.pclk1.0 * if self.ppre1() == 1 { 1 } else { 2 })
    }

    /// Returns the frequency of the APB2 Timers
    pub fn pclk2_tim(&self) -> Hertz {
        Hertz(self.pclk2.0 * if self.ppre2() == 1 { 1 } else { 2 })
    }

    pub(crate) fn ppre1(&self) -> u8 {
        self.ppre1
    }

    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn ppre2(&self) -> u8 {
        self.ppre2
    }

    /// Returns the system (core) frequency
    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }

    /// Returns the adc clock frequency
    pub fn adcclk(&self) -> Hertz {
        self.adcclk
    }

    /// Returns whether the USBCLK clock frequency is valid for the USB peripheral
    pub fn usbclk_valid(&self) -> bool {
        self.usbclk_valid
    }
}

pub trait GetBusFreq {
    fn get_frequency(clocks: &Clocks) -> Hertz;
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        Self::get_frequency(clocks)
    }
}

impl GetBusFreq for AHB {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.hclk
    }
}

impl GetBusFreq for APB1 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.pclk1
    }
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        clocks.pclk1_tim()
    }
}

impl GetBusFreq for APB2 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.pclk2
    }
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        clocks.pclk2_tim()
    }
}

pub(crate) mod sealed {
    /// Bus associated to peripheral
    pub trait RcuBus {
        /// Bus type;
        type Bus;
    }
}
use sealed::RcuBus;

/// Enable/disable peripheral
pub trait Enable: RcuBus {
    fn enable(apb: &mut Self::Bus);
    fn disable(apb: &mut Self::Bus);
}

/// Reset peripheral
pub trait Reset: RcuBus {
    fn reset(apb: &mut Self::Bus);
}

macro_rules! bus {
    ($($PER:ident => ($apbX:ty, $peren:ident, $perrst:ident),)+) => {
        $(
            impl RcuBus for crate::pac::$PER {
                type Bus = $apbX;
            }
            impl Enable for crate::pac::$PER {
                #[inline(always)]
                fn enable(apb: &mut Self::Bus) {
                    apb.enr().modify(|_, w| w.$peren().set_bit());
                }
                #[inline(always)]
                fn disable(apb: &mut Self::Bus) {
                    apb.enr().modify(|_, w| w.$peren().clear_bit());
                }
            }
            impl Reset for crate::pac::$PER {
                #[inline(always)]
                fn reset(apb: &mut Self::Bus) {
                    apb.rstr().modify(|_, w| w.$perrst().set_bit());
                    apb.rstr().modify(|_, w| w.$perrst().clear_bit());
                }
            }
        )+
    }
}

macro_rules! ahb_bus {
    ($($PER:ident => ($peren:ident),)+) => {
        $(
            impl RcuBus for crate::pac::$PER {
                type Bus = AHB;
            }
            impl Enable for crate::pac::$PER {
                #[inline(always)]
                fn enable(apb: &mut Self::Bus) {
                    apb.enr().modify(|_, w| w.$peren().set_bit());
                }
                #[inline(always)]
                fn disable(apb: &mut Self::Bus) {
                    apb.enr().modify(|_, w| w.$peren().clear_bit());
                }
            }
        )+
    }
}

bus! {
    ADC => (APB2, adcen, adcrst),
    CAN0 => (APB1, can0en, can0rst),
    CAN1 => (APB1, can1en, can1rst),
    I2C0 => (APB1, i2c0en, i2c0rst),
    I2C1 => (APB1, i2c1en, i2c1rst),
    //I2C2 => (ADDAPB1, i2c2en, i2c2rst),
    SPI0 => (APB2, spi0en, spi0rst),
    SPI1 => (APB1, spi1en, spi1rst),
    SPI2 => (APB1, spi2en, spi2rst),
    TIMER0 => (APB2, timer0en, timer0rst),
    TIMER1 => (APB1, timer1en, timer1rst),
    TIMER2 => (APB1, timer2en, timer2rst),
    TIMER5 => (APB1, timer5en, timer5rst),
    USART0 => (APB2, usart0en, usart0rst),
    USART1 => (APB1, usart1en, usart1rst),
    USBD => (APB1, usbden, usbdrst),
    WWDGT => (APB1, wwdgten, wwdgtrst),
}

ahb_bus! {
    CRC => (crcen),
    DMA => (dmaen),
    GPIOA => (paen),
    GPIOB => (pben),
    GPIOC => (pcen),
    GPIOD => (pden),
    GPIOF => (pfen),
}
