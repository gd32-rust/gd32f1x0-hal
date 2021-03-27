use crate::rcu::Clocks;
use crate::time::Hertz;
use cast::{u32, u64};
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

impl Timer<SYST> {
    pub fn syst<T>(mut syst: SYST, clocks: &Clocks) -> Self
    where
        T: Into<Hertz>,
    {
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
