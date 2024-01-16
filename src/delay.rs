// Copyright 2021 The gd32f1x0-hal authors.
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! # Delays

use crate::rcu::Clocks;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use embedded_hal::delay::DelayNs;
use embedded_hal_02::blocking::delay::{DelayMs, DelayUs};

/// System timer (SysTick) as a delay provider
pub struct Delay {
    syst: SYST,
    clocks: Clocks,
}

impl Delay {
    /// Configures the system timer (SysTick) as a delay provider
    pub fn new(mut syst: SYST, clocks: Clocks) -> Self {
        syst.set_clock_source(SystClkSource::Core);

        Delay { syst, clocks }
    }

    /// Releases the system timer (SysTick) resource
    pub fn free(self) -> SYST {
        self.syst
    }
}

impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        // The SysTick Reload Value register supports values between 1 and 0x00FFFFFF.
        const MAX_RVR: u32 = 0x00FF_FFFF;

        let mut total_ticks = ns
            .wrapping_mul(self.clocks.hclk().0)
            .wrapping_div(1_000_000_000);

        while total_ticks != 0 {
            let current_rvr = if total_ticks <= MAX_RVR {
                total_ticks
            } else {
                MAX_RVR
            };

            self.syst.set_reload(current_rvr);
            self.syst.clear_current();
            self.syst.enable_counter();

            // Update the tracking variable while we are waiting...
            total_ticks -= current_rvr;

            while !self.syst.has_wrapped() {}

            self.syst.disable_counter();
        }
    }
}

impl DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        DelayNs::delay_ms(self, ms);
    }
}

impl DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        DelayNs::delay_ms(self, ms.into());
    }
}

impl DelayMs<u8> for Delay {
    fn delay_ms(&mut self, ms: u8) {
        DelayNs::delay_ms(self, ms.into());
    }
}

impl DelayUs<u32> for Delay {
    fn delay_us(&mut self, us: u32) {
        DelayNs::delay_us(self, us)
    }
}

impl DelayUs<u16> for Delay {
    fn delay_us(&mut self, us: u16) {
        DelayNs::delay_us(self, us.into())
    }
}

impl DelayUs<u8> for Delay {
    fn delay_us(&mut self, us: u8) {
        DelayNs::delay_us(self, us.into())
    }
}
