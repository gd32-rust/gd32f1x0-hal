// Copyright 2021 The gd32f1x0-hal authors.
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! # Reset & Clock Unit

use crate::flash::WS;
#[cfg(any(feature = "gd32f130", feature = "gd32f150"))]
use crate::pac::rcu::cfg0::USBDPSC_A;
use crate::pac::{
    fmc::ws::WSCNT_A,
    rcu::{
        self,
        cfg0::{ADCPSC_A, AHBPSC_A, APB1PSC_A, APB2PSC_A, PLLSEL_A, SCS_A},
    },
    RCU,
};
use crate::time::Hertz;
use cast::u32;
use core::cmp;

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
                hxtal: None,
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

/// Frequency of the internal 8MHz RC oscillator in Hz
const IRC8M: u32 = 8_000_000;

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
    hxtal: Option<u32>,
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
    adcclk: Option<u32>,
}

impl CFGR {
    /// Uses HXTAL (external oscillator) instead of IRC8M (internal RC oscillator) as the clock source.
    /// Will result in a hang if an external oscillator is not connected or it fails to start.
    /// The frequency specified must be the frequency of the external oscillator
    pub fn use_hxtal<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hxtal = Some(freq.into().0);
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
    pub fn freeze(self, ws: &mut WS) -> Clocks {
        let pllsrculk = self.hxtal.unwrap_or(IRC8M / 2);

        let pllmul = self.sysclk.unwrap_or(pllsrculk) / pllsrculk;

        let (pllmf_msb, pllmf_bits, sysclk) = if pllmul == 1 {
            (false, None, self.hxtal.unwrap_or(IRC8M))
        } else {
            let pllmul = cmp::min(cmp::max(pllmul, 2), 32);
            if pllmul > 16 {
                (true, Some(pllmul as u8 - 17), pllsrculk * pllmul)
            } else {
                (false, Some(pllmul as u8 - 2), pllsrculk * pllmul)
            }
        };

        assert!(sysclk <= 72_000_000);

        let (ahbpsc, hclk) = match sysclk / self.hclk.unwrap_or(sysclk) {
            0 => unreachable!(),
            1 => (AHBPSC_A::DIV1, sysclk),
            2 => (AHBPSC_A::DIV2, sysclk / 2),
            3..=5 => (AHBPSC_A::DIV4, sysclk / 4),
            6..=11 => (AHBPSC_A::DIV8, sysclk / 8),
            12..=39 => (AHBPSC_A::DIV16, sysclk / 16),
            40..=95 => (AHBPSC_A::DIV64, sysclk / 64),
            96..=191 => (AHBPSC_A::DIV128, sysclk / 128),
            192..=383 => (AHBPSC_A::DIV256, sysclk / 256),
            _ => (AHBPSC_A::DIV512, sysclk / 512),
        };

        assert!(hclk <= 72_000_000);

        // Default APB1 clock to less than 36 MHz so I2C will work.
        let (apb1psc, ppre1) = match hclk / self.pclk1.unwrap_or_else(|| cmp::min(hclk, 36_000_000))
        {
            0 => unreachable!(),
            1 => (APB1PSC_A::DIV1, 1),
            2 => (APB1PSC_A::DIV2, 2),
            3..=5 => (APB1PSC_A::DIV4, 4),
            6..=11 => (APB1PSC_A::DIV8, 8),
            _ => (APB1PSC_A::DIV16, 16),
        };

        let pclk1 = hclk / u32(ppre1);

        assert!(pclk1 <= 72_000_000);

        let (apb2psc, ppre2) = match hclk / self.pclk2.unwrap_or(hclk) {
            0 => unreachable!(),
            1 => (APB2PSC_A::DIV1, 1),
            2 => (APB2PSC_A::DIV2, 2),
            3..=5 => (APB2PSC_A::DIV4, 4),
            6..=11 => (APB2PSC_A::DIV8, 8),
            _ => (APB2PSC_A::DIV16, 16),
        };

        let pclk2 = hclk / u32(ppre2);

        assert!(pclk2 <= 72_000_000);

        // adjust flash wait states
        ws.ws().write(|w| {
            w.wscnt().variant(if sysclk <= 24_000_000 {
                WSCNT_A::WS0
            } else if sysclk <= 48_000_000 {
                WSCNT_A::WS1
            } else {
                WSCNT_A::WS2
            })
        });

        #[cfg(any(feature = "gd32f130", feature = "gd32f150"))]
        // the USB clock is only valid if an external crystal is used, the PLL is enabled, and the
        // PLL output frequency is a supported one.
        // usbpre == false: divide clock by 1.5, otherwise no division
        let (usbpre, usbclk_valid) = match (self.hxtal, pllmf_bits, sysclk) {
            (Some(_), Some(_), 72_000_000) => (USBDPSC_A::DIV1_5, true),
            (Some(_), Some(_), 48_000_000) => (USBDPSC_A::DIV1, true),
            _ => (USBDPSC_A::DIV1_5, false),
        };

        let (adcpsc, adcclk) = match pclk2 / self.adcclk.unwrap_or(pclk2 / 8) {
            0..=2 => (ADCPSC_A::DIV2, pclk2 / 2),
            3..=4 => (ADCPSC_A::DIV4, pclk2 / 4),
            5..=7 => (ADCPSC_A::DIV6, pclk2 / 6),
            _ => (ADCPSC_A::DIV8, pclk2 / 8),
        };

        let rcu = unsafe { &*RCU::ptr() };

        if self.hxtal.is_some() {
            // Enable HXTAL and wait for it to be ready.
            rcu.ctl0.modify(|_, w| w.hxtalen().on());
            while rcu.ctl0.read().hxtalstb().is_not_ready() {}
        }

        if let Some(pllmf_bits) = pllmf_bits {
            // enable PLL and wait for it to be ready

            rcu.cfg0.modify(|_, w| {
                w.pllmf()
                    .bits(pllmf_bits)
                    .pllmf_msb()
                    .bit(pllmf_msb)
                    .pllsel()
                    .variant(if self.hxtal.is_some() {
                        PLLSEL_A::HXTAL
                    } else {
                        PLLSEL_A::IRC8M_2
                    })
            });

            rcu.ctl0.modify(|_, w| w.pllen().on());

            while rcu.ctl0.read().pllstb().is_not_ready() {}
        }

        // set prescalers and clock source
        rcu.cfg0.modify(|_, w| {
            w.adcpsc()
                .variant(adcpsc)
                .apb2psc()
                .variant(apb2psc)
                .apb1psc()
                .variant(apb1psc)
                .ahbpsc()
                .variant(ahbpsc)
                .scs()
                .variant(if pllmf_bits.is_some() {
                    SCS_A::PLL
                } else if self.hxtal.is_some() {
                    SCS_A::HXTAL
                } else {
                    SCS_A::IRC8M
                })
        });
        #[cfg(any(feature = "gd32f130", feature = "gd32f150"))]
        {
            rcu.cfg0.modify(|_, w| w.usbdpsc().variant(usbpre));
        }
        rcu.cfg2.modify(|_, w| w.adcsel().apb2().usart0sel().apb2());

        Clocks {
            hclk: Hertz(hclk),
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            ppre1,
            ppre2,
            sysclk: Hertz(sysclk),
            adcclk: Hertz(adcclk),
            #[cfg(any(feature = "gd32f130", feature = "gd32f150"))]
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
    #[cfg(any(feature = "gd32f130", feature = "gd32f150"))]
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
    #[cfg(any(feature = "gd32f130", feature = "gd32f150"))]
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
                    apb.enr().modify(|_, w| w.$peren().enabled());
                }
                #[inline(always)]
                fn disable(apb: &mut Self::Bus) {
                    apb.enr().modify(|_, w| w.$peren().disabled());
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
                    apb.enr().modify(|_, w| w.$peren().enabled());
                }
                #[inline(always)]
                fn disable(apb: &mut Self::Bus) {
                    apb.enr().modify(|_, w| w.$peren().disabled());
                }
            }
        )+
    }
}

bus! {
    ADC => (APB2, adcen, adcrst),
    I2C0 => (APB1, i2c0en, i2c0rst),
    I2C1 => (APB1, i2c1en, i2c1rst),
    // TODO: Support I2C2 on GD32F170/GD32F190
    //I2C2 => (ADDAPB1, i2c2en, i2c2rst),
    SPI0 => (APB2, spi0en, spi0rst),
    SPI1 => (APB1, spi1en, spi1rst),
    SPI2 => (APB1, spi2en, spi2rst),
    TIMER0 => (APB2, timer0en, timer0rst),
    TIMER1 => (APB1, timer1en, timer1rst),
    TIMER2 => (APB1, timer2en, timer2rst),
    TIMER5 => (APB1, timer5en, timer5rst),
    TIMER13 => (APB1, timer13en, timer13rst),
    TIMER14 => (APB2, timer14en, timer14rst),
    TIMER15 => (APB2, timer15en, timer15rst),
    TIMER16 => (APB2, timer16en, timer16rst),
    USART0 => (APB2, usart0en, usart0rst),
    USART1 => (APB1, usart1en, usart1rst),
    WWDGT => (APB1, wwdgten, wwdgtrst),
}

#[cfg(feature = "gd32f150")]
bus! {
    USBD => (APB1, usbden, usbdrst),
}

#[cfg(any(feature = "gd32f170", feature = "gd32f190"))]
bus! {
    CAN0 => (APB1, can0en, can0rst),
    CAN1 => (APB1, can1en, can1rst),
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
