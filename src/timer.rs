// Copyright 2021 The gd32f1x0-hal authors.
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use crate::pac::{
    dbg::ctl0::TIMER0_HOLD_A, dbg::ctl1::TIMER14_HOLD_A, DBG, TIMER0, TIMER1, TIMER13, TIMER14,
    TIMER15, TIMER16, TIMER2, TIMER5,
};
use crate::rcu::{sealed::RcuBus, Clocks, Enable, GetBusFreq, Reset, APB1, APB2};
use crate::time::Hertz;
use cast::{u16, u32, u64};
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use embedded_hal::timer::{Cancel, CountDown, Periodic};
use void::Void;

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    Update,
}

#[derive(Debug, PartialEq)]
pub enum Error {
    /// Timer is canceled
    Canceled,
}

pub struct Timer<TIMER> {
    pub(crate) timer: TIMER,
    pub(crate) clock: Hertz,
}

pub struct CountDownTimer<TIMER> {
    timer: TIMER,
    clock: Hertz,
}

pub enum DebugHold {
    /// Continue running even when stopped for debugging.
    Continue,
    /// Stop the timer when stopped for debugging.
    Stop,
}

impl From<DebugHold> for TIMER0_HOLD_A {
    fn from(hold: DebugHold) -> Self {
        match hold {
            DebugHold::Continue => TIMER0_HOLD_A::CONTINUE,
            DebugHold::Stop => TIMER0_HOLD_A::STOP,
        }
    }
}

impl From<DebugHold> for TIMER14_HOLD_A {
    fn from(hold: DebugHold) -> Self {
        match hold {
            DebugHold::Continue => TIMER14_HOLD_A::CONTINUE,
            DebugHold::Stop => TIMER14_HOLD_A::STOP,
        }
    }
}

impl Timer<SYST> {
    pub fn syst(mut syst: SYST, clocks: &Clocks) -> Self {
        syst.set_clock_source(SystClkSource::Core);
        Self {
            timer: syst,
            // TODO: Do we need to divide this by 8?
            clock: clocks.hclk(),
        }
    }

    /// Configures the SYST timer as a periodic count down timer.
    pub fn start_count_down<T>(self, timeout: T) -> CountDownTimer<SYST>
    where
        T: Into<Hertz>,
    {
        let Self { timer, clock } = self;
        let mut timer = CountDownTimer { timer, clock };
        timer.start(timeout);
        timer
    }

    pub fn release(self) -> SYST {
        self.timer
    }
}

impl CountDownTimer<SYST> {
    /// Starts listening for an `event`
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Update => self.timer.enable_interrupt(),
        }
    }

    /// Stops listening for an `event`
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Update => self.timer.disable_interrupt(),
        }
    }

    /// Resets the counter
    pub fn reset(&mut self) {
        // According to the Cortex-M3 Generic User Guide, the interrupt request is only generated
        // when the counter goes from 1 to 0, so writing zero should not trigger an interrupt
        self.timer.clear_current();
    }

    /// Returns the number of microseconds since the last update event.
    /// *NOTE:* This method is not a very good candidate to keep track of time, because
    /// it is very easy to lose an update event.
    pub fn micros_since(&self) -> u32 {
        let reload_value = SYST::get_reload();
        let timer_clock = u64(self.clock.0);
        let ticks = u64(reload_value - SYST::get_current());

        // It is safe to make this cast since the maximum ticks is (2^24 - 1) and the minimum sysclk
        // is 4Mhz, which gives a maximum period of ~4.2 seconds which is < (2^32 - 1) microseconds
        u32(1_000_000 * ticks / timer_clock).unwrap()
    }

    /// Stops the timer
    pub fn stop(mut self) -> Timer<SYST> {
        self.timer.disable_counter();
        let Self { timer, clock } = self;
        Timer { timer, clock }
    }

    /// Releases the SYST
    pub fn release(self) -> SYST {
        self.stop().release()
    }
}

impl CountDown for CountDownTimer<SYST> {
    type Time = Hertz;

    fn start<T>(&mut self, timeout: T)
    where
        T: Into<Hertz>,
    {
        let rvr = self.clock.0 / timeout.into().0 - 1;

        assert!(rvr < (1 << 24));

        self.timer.set_reload(rvr);
        self.timer.clear_current();
        self.timer.enable_counter();
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        if self.timer.has_wrapped() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl Cancel for CountDownTimer<SYST> {
    type Error = Error;

    fn cancel(&mut self) -> Result<(), Self::Error> {
        if !self.timer.is_counter_enabled() {
            return Err(Self::Error::Canceled);
        }

        self.timer.disable_counter();
        Ok(())
    }
}

impl Periodic for CountDownTimer<SYST> {}

/// Helper methods used by other parts of the HAL, such as PWM.
pub(crate) trait TimerExt {
    /// Resets the counter by triggering an update event, disabling the interrupt while doing so.
    fn reset_counter(&mut self);
    /// Configures the timer with an appropriate prescaler and reload value to count down at the
    /// given timeout frequency, assuming it is using the given clock frequency.
    fn configure_prescaler_reload(&mut self, timeout: Hertz, clock: Hertz);
}

macro_rules! hal {
    ($TIMERX:ident: ($timerX:ident, $APBx:ident, $dbg_ctlX:ident, $timerX_hold:ident$(,$master_timerbase:ident)*)) => {
        impl Timer<$TIMERX> {
            /// Initializes the timer.
            pub fn $timerX(timer: $TIMERX, clocks: &Clocks, apb: &mut $APBx) -> Self {
                // Enable and reset peripheral to a clean state.
                $TIMERX::enable(apb);
                $TIMERX::reset(apb);

                Self {
                    timer,
                    clock: <$TIMERX as RcuBus>::Bus::get_timer_frequency(clocks),
                }
            }

            /// Starts the timer in count down mode at a given frequency.
            pub fn start_count_down<T>(self, timeout: T) -> CountDownTimer<$TIMERX>
            where
                T: Into<Hertz>,
            {
                let Self { timer, clock } = self;
                let mut timer = CountDownTimer { timer, clock };
                timer.start(timeout);
                timer
            }

            $(
                /// Starts timer in count down mode at a given frequency, and additionally configures
                /// the timer's master mode.
                pub fn start_master<T>(
                    self,
                    timeout: T,
                    mode: crate::pac::$master_timerbase::ctl1::MMC_A,
                ) -> CountDownTimer<$TIMERX>
                where
                    T: Into<Hertz>,
                {
                    let Self { timer, clock } = self;
                    let mut timer = CountDownTimer { timer, clock };
                    timer.timer.ctl1.modify(|_, w| w.mmc().variant(mode));
                    timer.start(timeout);
                    timer
                }
            )?

            /// Resets the timer peripheral.
            #[inline(always)]
            pub fn clocking_reset(&mut self, apb: &mut <$TIMERX as RcuBus>::Bus) {
                $TIMERX::reset(apb);
            }

            /// Configures whether the timer will be stopped when stopping for debugging.
            ///
            /// Stopping timer in debug mode can cause troubles when sampling the signal.
            #[inline(always)]
            pub fn stop_in_debug(&mut self, dbg: &mut DBG, hold: DebugHold) {
                dbg.$dbg_ctlX.modify(|_, w| w.$timerX_hold().variant(hold.into()));
            }

            /// Releases the TIMER Peripheral.
            pub fn release(self) -> $TIMERX {
                // TODO: Disable timer?
                self.timer
            }
        }

        impl CountDownTimer<$TIMERX> {
            /// Starts listening for an `event`.
            pub fn listen(&mut self, event: Event) {
                match event {
                    Event::Update => self.timer.dmainten.modify(|_, w| w.upie().enabled()),
                }
            }

            /// Stops listening for an `event`.
            pub fn unlisten(&mut self, event: Event) {
                match event {
                    Event::Update => self.timer.dmainten.modify(|_, w| w.upie().disabled()),
                }
            }

            /// Stops the timer
            pub fn stop(self) -> Timer<$TIMERX> {
                self.timer.ctl0.modify(|_, w| w.cen().disabled());
                let Self { timer, clock } = self;
                Timer { timer, clock }
            }

            /// Clears the update interrupt flag.
            pub fn clear_update_interrupt_flag(&mut self) {
                self.timer.intf.modify(|_, w| w.upif().clear());
            }

            /// Releases the TIMER peripheral.
            pub fn release(self) -> $TIMERX {
                self.stop().release()
            }

            /// Returns the number of microseconds since the last update event.
            /// *NOTE:* This method is not a very good candidate to keep track of time, because
            /// it is very easy to lose an update event.
            pub fn micros_since(&self) -> u32 {
                let timer_clock = self.clock.0;
                let psc = u32(self.timer.psc.read().psc().bits());

                // freq_divider is always bigger than 0, since (psc + 1) is always less than
                // timer_clock
                let freq_divider = u64(timer_clock / (psc + 1));
                let cnt = u64(self.timer.cnt.read().cnt().bits());

                // It is safe to make this cast, because the maximum timer period in this HAL is
                // 1s (1Hz), then 1 second < (2^32 - 1) microseconds
                u32(1_000_000 * cnt / freq_divider).unwrap()
            }

            /// Resets the counter
            pub fn reset(&mut self) {
                self.timer.reset_counter();
            }
        }

        impl TimerExt for $TIMERX {
            fn reset_counter(&mut self) {
                // Sets the UPS bit to prevent an interrupt from being triggered by the UPG bit.
                self.ctl0.modify(|_, w| w.ups().counter_only());

                self.swevg.write(|w| w.upg().update());
                self.ctl0.modify(|_, w| w.ups().any_event());
            }

            fn configure_prescaler_reload(&mut self, timeout: Hertz, clock: Hertz) {
                // Calculate prescaler and reload values.
                let (prescaler, auto_reload_value) = compute_prescaler_reload(timeout, clock);
                self.psc.write(|w| w.psc().bits(prescaler));
                // TODO: Support 32-bit counters
                self.car.write(|w| w.car().bits(auto_reload_value.into()));
            }
        }

        impl CountDown for CountDownTimer<$TIMERX> {
            type Time = Hertz;

            fn start<T>(&mut self, timeout: T)
            where
                T: Into<Hertz>,
            {
                // Pause counter.
                self.timer.ctl0.modify(|_, w| w.cen().disabled());

                self.timer.configure_prescaler_reload(timeout.into(), self.clock);
                // Trigger an update event to load the prescaler value to the clock
                self.timer.reset_counter();

                // Start counter.
                self.timer.ctl0.modify(|_, w| w.cen().enabled());
            }

            fn wait(&mut self) -> nb::Result<(), Void> {
                if self.timer.intf.read().upif().is_clear() {
                    Err(nb::Error::WouldBlock)
                } else {
                    self.clear_update_interrupt_flag();
                    Ok(())
                }
            }
        }

        impl Cancel for CountDownTimer<$TIMERX> {
            type Error = Error;

            fn cancel(&mut self) -> Result<(), Self::Error> {
                let is_counter_enabled = self.timer.ctl0.read().cen().is_enabled();
                if !is_counter_enabled {
                    return Err(Self::Error::Canceled);
                }

                // Pause counter.
                self.timer.ctl0.modify(|_, w| w.cen().disabled());
                Ok(())
            }
        }

        impl Periodic for CountDownTimer<$TIMERX> {}
    };
}

#[inline(always)]
fn compute_prescaler_reload(freq: Hertz, clock: Hertz) -> (u16, u16) {
    let ticks = clock.0 / freq.0;
    let psc = u16((ticks - 1) >> 16).unwrap();
    let car = u16(ticks / u32(psc + 1)).unwrap();
    (psc, car)
}

hal!(TIMER0: (timer0, APB2, ctl0, timer0_hold, timer0));
hal!(TIMER1: (timer1, APB1, ctl0, timer1_hold, timer1));
hal!(TIMER2: (timer2, APB1, ctl0, timer2_hold, timer1));
hal!(TIMER5: (timer5, APB1, ctl0, timer5_hold, timer5));
hal!(TIMER13: (timer13, APB1, ctl0, timer13_hold));
hal!(TIMER14: (timer14, APB2, ctl1, timer14_hold, timer14));
hal!(TIMER15: (timer15, APB2, ctl1, timer15_hold));
hal!(TIMER16: (timer16, APB2, ctl1, timer16_hold));
