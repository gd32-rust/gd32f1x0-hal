// Copyright 2021 The gd32f1x0-hal authors.
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use crate::pac::{
    dbg::ctl0::Timer0Hold, dbg::ctl1::Timer14Hold, Dbg, Timer0, Timer1, Timer13, Timer14, Timer15,
    Timer16, Timer2, Timer5,
};
use crate::rcu::{sealed::RcuBus, Clocks, Enable, GetBusFreq, Reset, APB1, APB2};
use crate::time::Hertz;
use core::convert::TryFrom;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;

/// Interrupt events
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Event {
    /// Timer timed out / count down ended
    Update,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, thiserror::Error)]
pub enum Error {
    /// Timer is cancelled.
    #[error("Timer is cancelled")]
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

impl From<DebugHold> for Timer0Hold {
    fn from(hold: DebugHold) -> Self {
        match hold {
            DebugHold::Continue => Timer0Hold::Continue,
            DebugHold::Stop => Timer0Hold::Stop,
        }
    }
}

impl From<DebugHold> for Timer14Hold {
    fn from(hold: DebugHold) -> Self {
        match hold {
            DebugHold::Continue => Timer14Hold::Continue,
            DebugHold::Stop => Timer14Hold::Stop,
        }
    }
}

impl Timer<SYST> {
    pub fn syst(mut syst: SYST, clocks: &Clocks) -> Self {
        syst.set_clock_source(SystClkSource::Core);
        Self {
            timer: syst,
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
        let timer_clock = u64::from(self.clock.0);
        let ticks = u64::from(reload_value - SYST::get_current());

        // It is safe to make this cast since the maximum ticks is (2^24 - 1) and the minimum sysclk
        // is 4Mhz, which gives a maximum period of ~4.2 seconds which is < (2^32 - 1) microseconds
        u32::try_from(1_000_000 * ticks / timer_clock).unwrap()
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

    /// Configures the timer to have the given timeout and enables it to start counting down.
    pub fn start<T>(&mut self, timeout: T)
    where
        T: Into<Hertz>,
    {
        let rvr = self.clock.0 / timeout.into().0 - 1;

        assert!(rvr < (1 << 24));

        self.timer.set_reload(rvr);
        self.timer.clear_current();
        self.timer.enable_counter();
    }

    /// Returns whether the timer has finished counting down yet, and resets the flag if so.
    pub fn has_elapsed(&mut self) -> bool {
        self.timer.has_wrapped()
    }

    /// Disables the timer.
    pub fn cancel(&mut self) -> Result<(), Error> {
        if !self.timer.is_counter_enabled() {
            return Err(Error::Canceled);
        }

        self.timer.disable_counter();
        Ok(())
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::timer::CountDown for CountDownTimer<SYST> {
    type Time = Hertz;

    fn start<T>(&mut self, timeout: T)
    where
        T: Into<Hertz>,
    {
        self.start(timeout);
    }

    fn wait(&mut self) -> nb::Result<(), void::Void> {
        if self.has_elapsed() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::timer::Cancel for CountDownTimer<SYST> {
    type Error = Error;

    fn cancel(&mut self) -> Result<(), Self::Error> {
        self.cancel()
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::timer::Periodic for CountDownTimer<SYST> {}

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
                    mode: crate::pac::$master_timerbase::ctl1::Mmc,
                ) -> CountDownTimer<$TIMERX>
                where
                    T: Into<Hertz>,
                {
                    let Self { timer, clock } = self;
                    let mut timer = CountDownTimer { timer, clock };
                    timer.timer.ctl1().modify(|_, w| w.mmc().variant(mode));
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
            pub fn stop_in_debug(&mut self, dbg: &mut Dbg, hold: DebugHold) {
                dbg.$dbg_ctlX().modify(|_, w| w.$timerX_hold().variant(hold.into()));
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
                    Event::Update => self.timer.dmainten().modify(|_, w| w.upie().enabled()),
                }
            }

            /// Stops listening for an `event`.
            pub fn unlisten(&mut self, event: Event) {
                match event {
                    Event::Update => self.timer.dmainten().modify(|_, w| w.upie().disabled()),
                }
            }

            /// Stops the timer
            pub fn stop(self) -> Timer<$TIMERX> {
                self.timer.ctl0().modify(|_, w| w.cen().disabled());
                let Self { timer, clock } = self;
                Timer { timer, clock }
            }

            /// Returns true if the given `event` interrupt is pending.
            pub fn is_pending(&self, event: Event) -> bool {
                match event {
                    Event::Update => self.timer.intf().read().upif().is_update_pending(),
                }
            }

            /// Clears the given event interrupt flag.
            pub fn clear_interrupt_flag(&mut self, event: Event) {
                match event {
                    Event::Update => self.timer.intf().modify(|_, w| w.upif().clear()),
                }
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
                let psc = u32::from(self.timer.psc().read().psc().bits());

                // freq_divider is always bigger than 0, since (psc + 1) is always less than
                // timer_clock
                let freq_divider = u64::from(timer_clock / (psc + 1));
                let cnt = u64::from(self.timer.cnt().read().cnt().bits());

                // It is safe to make this cast, because the maximum timer period in this HAL is
                // 1s (1Hz), then 1 second < (2^32 - 1) microseconds
                u32::try_from(1_000_000 * cnt / freq_divider).unwrap()
            }

            /// Resets the counter
            pub fn reset(&mut self) {
                self.timer.reset_counter();
            }

            /// Configures the timer to have the given timeout and enables it to start counting down.
            pub fn start<T>(&mut self, timeout: T)
            where
                T: Into<Hertz>,
            {
                // Pause counter.
                self.timer.ctl0().modify(|_, w| w.cen().disabled());

                self.timer.configure_prescaler_reload(timeout.into(), self.clock);
                // Trigger an update event to load the prescaler value to the clock
                self.timer.reset_counter();

                // Start counter.
                self.timer.ctl0().modify(|_, w| w.cen().enabled());
            }

            /// Disables the timer.
            pub fn cancel(&mut self) -> Result<(), Error> {
                let is_counter_enabled = self.timer.ctl0().read().cen().is_enabled();
                if !is_counter_enabled {
                    return Err(Error::Canceled);
                }

                // Pause counter.
                self.timer.ctl0().modify(|_, w| w.cen().disabled());
                Ok(())
            }
        }

        impl TimerExt for $TIMERX {
            fn reset_counter(&mut self) {
                // Sets the UPS bit to prevent an interrupt from being triggered by the UPG bit.
                self.ctl0().modify(|_, w| w.ups().counter_only());

                self.swevg().write(|w| w.upg().update());
                self.ctl0().modify(|_, w| w.ups().any_event());
            }

            fn configure_prescaler_reload(&mut self, timeout: Hertz, clock: Hertz) {
                // Calculate prescaler and reload values.
                let (prescaler, auto_reload_value) = compute_prescaler_reload(timeout, clock);
                self.psc().write(|w| w.psc().bits(prescaler));
                // TODO: Support 32-bit counters
                self.car().write(|w| w.car().bits(auto_reload_value.into()));
            }
        }

        #[cfg(feature = "embedded-hal-02")]
        impl embedded_hal_02::timer::CountDown for CountDownTimer<$TIMERX> {
            type Time = Hertz;

            fn start<T>(&mut self, timeout: T)
            where
                T: Into<Hertz>,
            {
                self.start(timeout);
            }

            fn wait(&mut self) -> nb::Result<(), void::Void> {
                if !self.is_pending(Event::Update) {
                    Err(nb::Error::WouldBlock)
                } else {
                    self.clear_interrupt_flag(Event::Update);
                    Ok(())
                }
            }
        }

        #[cfg(feature = "embedded-hal-02")]
        impl embedded_hal_02::timer::Cancel for CountDownTimer<$TIMERX> {
            type Error = Error;

            fn cancel(&mut self) -> Result<(), Self::Error> {
                self.cancel()
            }
        }

        #[cfg(feature = "embedded-hal-02")]
        impl embedded_hal_02::timer::Periodic for CountDownTimer<$TIMERX> {}
    };
}

#[inline(always)]
fn compute_prescaler_reload(freq: Hertz, clock: Hertz) -> (u16, u16) {
    let ticks = clock.0 / freq.0;
    let psc = u16::try_from((ticks - 1) >> 16).unwrap();
    let car = u16::try_from(ticks / u32::from(psc + 1)).unwrap();
    (psc, car)
}

hal!(Timer0: (timer0, APB2, ctl0, timer0_hold, timer0));
hal!(Timer1: (timer1, APB1, ctl0, timer1_hold, timer1));
hal!(Timer2: (timer2, APB1, ctl0, timer2_hold, timer1));
hal!(Timer5: (timer5, APB1, ctl0, timer5_hold, timer5));
hal!(Timer13: (timer13, APB1, ctl0, timer13_hold));
hal!(Timer14: (timer14, APB2, ctl1, timer14_hold, timer14));
hal!(Timer15: (timer15, APB2, ctl1, timer15_hold));
hal!(Timer16: (timer16, APB2, ctl1, timer16_hold));
