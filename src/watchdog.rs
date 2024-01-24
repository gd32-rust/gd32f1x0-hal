// Copyright 2021 The gd32f1x0-hal authors.
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Watchdog peripherals

use crate::{
    pac::{DBG, FWDGT},
    time::MilliSeconds,
};

/// Wraps the Free Watchdog (FWDGT) peripheral
pub struct FreeWatchdog {
    fwdgt: FWDGT,
}

const LSI_KHZ: u32 = 40;
const MAX_PRESCALER: u8 = 8;
const MAX_RELOAD: u16 = 0xFFF;

impl FreeWatchdog {
    /// Wrap and start the watchdog
    pub fn new(fwdgt: FWDGT) -> Self {
        FreeWatchdog { fwdgt }
    }

    /// Debug free watchdog stopped when core is halted
    pub fn stop_on_debug(&self, dbg: &DBG, stop: bool) {
        dbg.ctl0.modify(|_, w| w.fwdgt_hold().bit(stop));
    }

    fn setup(&self, timeout_ms: u32) {
        let mut prescaler = 0;
        while prescaler < MAX_PRESCALER && Self::timeout_period(prescaler, MAX_RELOAD) < timeout_ms
        {
            prescaler += 1;
        }

        let max_period = Self::timeout_period(prescaler, MAX_RELOAD);
        let max_reload = u32::from(MAX_RELOAD);
        let reload = (timeout_ms * max_reload / max_period).min(max_reload) as u16;

        self.access_registers(|fwdgt| {
            fwdgt.psc.modify(|_, w| w.psc().bits(prescaler));
            fwdgt.rld.modify(|_, w| w.rld().bits(reload));
        });
    }

    fn is_prescaler_updating(&self) -> bool {
        self.fwdgt.stat.read().pud().is_ongoing()
    }

    /// Returns the interval in ms
    pub fn interval(&self) -> MilliSeconds {
        while self.is_prescaler_updating() {}

        let prescaler = self.fwdgt.psc.read().psc().bits();
        let reload = self.fwdgt.rld.read().rld().bits();
        let ms = Self::timeout_period(prescaler, reload);
        MilliSeconds(ms)
    }

    /// prescaler: Prescaler divider bits, reload: reload value
    ///
    /// Returns ms
    fn timeout_period(prescaler: u8, reload: u16) -> u32 {
        let divider: u32 = match prescaler {
            0b000 => 4,
            0b001 => 8,
            0b010 => 16,
            0b011 => 32,
            0b100 => 64,
            0b101 => 128,
            0b110 => 256,
            0b111 => 256,
            _ => panic!("Invalid FWDGT prescaler divider"),
        };
        (u32::from(reload) + 1) * divider / LSI_KHZ
    }

    fn access_registers<A, F: FnMut(&FWDGT) -> A>(&self, mut f: F) -> A {
        // Unprotect write access to registers
        self.fwdgt.ctl.write(|w| w.cmd().enable());
        let a = f(&self.fwdgt);

        // Protect again
        self.fwdgt.ctl.write(|w| w.cmd().reset());
        a
    }

    /// Configures the watchdog to use the given period and enables it.
    pub fn start<T: Into<MilliSeconds>>(&mut self, period: T) {
        self.setup(period.into().0);

        self.fwdgt.ctl.write(|w| w.cmd().start());
    }

    /// Resets the watchdog.
    ///
    /// This must be done periodically once the watchdog is started to prevent the processor being
    /// reset.
    pub fn feed(&mut self) {
        self.fwdgt.ctl.write(|w| w.cmd().reset());
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::watchdog::WatchdogEnable for FreeWatchdog {
    type Time = MilliSeconds;

    fn start<T: Into<Self::Time>>(&mut self, period: T) {
        self.start(period);
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::watchdog::Watchdog for FreeWatchdog {
    fn feed(&mut self) {
        self.feed();
    }
}
