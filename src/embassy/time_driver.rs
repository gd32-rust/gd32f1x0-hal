use crate::pac::{Interrupt, interrupt};
use crate::rcu::{Clocks, Enable, GetBusFreq, Reset, sealed::RcuBus};
use core::cell::{Cell, RefCell};
use core::convert::TryInto;
use core::sync::atomic::{AtomicU32, Ordering, compiler_fence};
use core::task::Waker;
use cortex_m::peripheral::NVIC;
use critical_section::CriticalSection;
use embassy_sync::{blocking_mutex::CriticalSectionMutex, once_lock::OnceLock};
use embassy_time_driver::{Driver, TICK_HZ, time_driver_impl};
use embassy_time_queue_utils::Queue;

#[cfg(not(any(
    feature = "time-driver-tim1",
    feature = "time-driver-tim2",
    feature = "time-driver-tim14",
)))]
compile_error!(
    "Embassy feature enabled however a time driver was not specified. A `--features time-driver-tim<1, 2, 14>` is required."
);

#[cfg(any(
    all(feature = "time-driver-tim1", feature = "time-driver-tim2"),
    all(feature = "time-driver-tim1", feature = "time-driver-tim14"),
    all(feature = "time-driver-tim2", feature = "time-driver-tim14"),
))]
compile_error!(
    "Multiple Embassy time drivers are specified. Only a single `--features time-driver-tim<1, 2, 14>` can be specified."
);

#[cfg(feature = "time-driver-tim1")]
pub(super) type Timer = crate::pac::Timer1;
#[cfg(feature = "time-driver-tim2")]
pub(super) type Timer = crate::pac::Timer2;
#[cfg(feature = "time-driver-tim14")]
pub(super) type Timer = crate::pac::Timer14;

#[cfg(any(feature = "time-driver-tim1", feature = "time-driver-tim2"))]
pub(super) type Bus = crate::rcu::APB1;
#[cfg(feature = "time-driver-tim14")]
pub(super) type Bus = crate::rcu::APB2;

#[cfg(any(feature = "time-driver-tim1", feature = "time-driver-tim2"))]
type RegisterBlock = crate::pac::timer1::RegisterBlock;
#[cfg(feature = "time-driver-tim14")]
type RegisterBlock = crate::pac::timer14::RegisterBlock;

struct AlarmState {
    timestamp: Cell<u64>,
}

impl AlarmState {
    const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
        }
    }
}

// NOTE: this definitely could be improved in the PAC crate
// An alternative would be to use a pointer-based access to the registers
trait ChannelsAccess {
    type Counter;

    fn set_chn_cc_value(&self, channel: usize, value: Self::Counter);
    fn set_chn_ie(&self, channel: usize, value: bool);
}

#[cfg(any(feature = "time-driver-tim1", feature = "time-driver-tim2"))]
impl ChannelsAccess for crate::pac::timer1::RegisterBlock {
    type Counter = u32;

    fn set_chn_cc_value(&self, channel: usize, value: Self::Counter) {
        match channel {
            0 => self.ch0cv().write(|w| w.ch0val().bits(value)),
            1 => self.ch1cv().write(|w| w.ch1val().bits(value)),
            2 => self.ch2cv().write(|w| w.ch2val().bits(value)),
            3 => self.ch3cv().write(|w| w.ch3val().bits(value)),
            _ => return,
        }
    }

    fn set_chn_ie(&self, channel: usize, value: bool) {
        match channel {
            0 => self.dmainten().modify(|_, w| w.ch0ie().bit(value)),
            1 => self.dmainten().modify(|_, w| w.ch1ie().bit(value)),
            2 => self.dmainten().modify(|_, w| w.ch2ie().bit(value)),
            3 => self.dmainten().modify(|_, w| w.ch3ie().bit(value)),
            _ => return,
        };
    }
}

#[cfg(feature = "time-driver-tim14")]
impl ChannelsAccess for crate::pac::timer14::RegisterBlock {
    type Counter = u16;

    fn set_chn_cc_value(&self, channel: usize, value: Self::Counter) {
        match channel {
            0 => self.ch0cv().write(|w| w.ch0val().bits(value)),
            1 => self.ch1cv().write(|w| w.ch1val().bits(value)),
            _ => return,
        }
    }

    fn set_chn_ie(&self, channel: usize, value: bool) {
        match channel {
            0 => self.dmainten().modify(|_, w| w.ch0ie().bit(value)),
            1 => self.dmainten().modify(|_, w| w.ch1ie().bit(value)),
            _ => return,
        };
    }
}

// Clock timekeeping works with something we call "periods", which are time intervals
// of 2^15 ticks. The Clock counter value is 16 bits, so one "overflow cycle" is 2 periods.
// Timer1 has 32 bits counter, however for a sake of consistency, only 16 bits are used.
//
// A `period` count is maintained in parallel to the Timer hardware `counter`, like this:
// - `period` and `counter` start at 0
// - `period` is incremented on overflow (at counter value 0)
// - `period` is incremented "midway" between overflows (at counter value 0x8000)
//
// Therefore, when `period` is even, counter is in 0..0x7FFF. When odd, counter is in 0x8000..0xFFFF
// This allows for now() to return the correct value even if it races an overflow.
//
// To get `now()`, `period` is read first, then `counter` is read. If the counter value matches
// the expected range for the `period` parity, we're done. If it doesn't, this means that
// a new period start has raced us between reading `period` and `counter`, so we assume the `counter` value
// corresponds to the next period.
//
// `period` is a 32bit unsigned integer, so It overflows on 2^32 * 2^15 / 32768 seconds of uptime, which is 136 years.
fn calc_now(period: u32, counter: u32) -> u64 {
    ((period as u64) << 15) + ((counter ^ ((period & 1) << 15)) as u64)
}

pub(super) struct EmbassyTimeDriver {
    /// Number of 2^31 periods elapsed since boot.
    period: AtomicU32,
    alarm: CriticalSectionMutex<AlarmState>,
    queue: CriticalSectionMutex<RefCell<Queue>>,
    timer: OnceLock<CriticalSectionMutex<Timer>>,
}

impl EmbassyTimeDriver {
    const fn new() -> Self {
        Self {
            period: AtomicU32::new(0),
            alarm: CriticalSectionMutex::new(AlarmState::new()),
            queue: CriticalSectionMutex::new(RefCell::new(Queue::new())),
            timer: OnceLock::new(),
        }
    }

    pub(super) fn init(timer: Timer, clocks: &Clocks, apb: &mut Bus) {
        Timer::enable(apb);
        Timer::reset(apb);

        timer.ctl0().modify(|_, w| w.cen().disabled());

        timer.cnt().write(|w| w.cnt().bits(0));

        let timer_freq = <Timer as RcuBus>::Bus::get_timer_frequency(clocks);
        // NOTE: using the default 1MHz tick rate
        let psc = (timer_freq.0 as u64) / TICK_HZ - 1;
        let psc: u16 = match psc.try_into() {
            Err(_) => panic!("psc division overflow: {}", psc),
            Ok(n) => n,
        };

        timer.psc().write(|w| w.psc().bits(psc));
        timer.car().write(|w| {
            w.car()
                .bits(u16::MAX as <RegisterBlock as ChannelsAccess>::Counter)
        });

        timer.ctl0().modify(|_, w| w.ups().counter_only());
        timer.swevg().write(|w| w.upg().update());
        timer.ctl0().modify(|_, w| w.ups().any_event());

        // Half of the u16::MAX value
        timer.ch0cv().write(|w| w.ch0val().bits(0x8000));

        timer
            .dmainten()
            .modify(|_, w| w.upie().enabled().ch0ie().enabled());

        timer.ctl0().modify(|_, w| w.cen().enabled());

        DRIVER.timer.init(CriticalSectionMutex::new(timer)).unwrap();

        // Don't unmask interrupt until after `DRIVER.timer` is initialised, to avoid race with
        // interrupt handler.
        #[cfg(feature = "time-driver-tim1")]
        {
            unsafe {
                NVIC::unmask(Interrupt::TIMER1);
            }
        }
        #[cfg(feature = "time-driver-tim2")]
        {
            unsafe {
                NVIC::unmask(Interrupt::TIMER2);
            }
        }
        #[cfg(feature = "time-driver-tim14")]
        {
            unsafe {
                NVIC::unmask(Interrupt::TIMER14);
            }
        }
    }

    fn on_interrupt(&self) {
        critical_section::with(|cs| {
            let timer = self.timer.try_get().unwrap().borrow(cs);
            let status = timer.intf().read();
            let enabled = timer.dmainten().read();

            // Clear all interrupt flags. Bits in SR are "write 0 to clear", so write the bitwise NOT.
            // Other approaches such as writing all zeros, or RMWing won't work, they can
            // miss interrupts.
            timer.intf().write(|w| unsafe { w.bits(!status.bits()) });

            if status.upif().bit() {
                self.next_period(timer)
            }

            if status.ch0if().bit() {
                self.next_period(timer)
            }

            // Bit 0 indicates Update CC interrupt, bit 1 corresponds to reserved Channel 0
            let channel = 2u8;
            if ((enabled.bits() >> channel) & 1 != 0) && ((status.bits() >> channel) & 1 != 0) {
                self.trigger_alarm(cs);
            }
        })
    }

    fn next_period(&self, timer: &Timer) {
        // We only modify the period from the timer interrupt, so we know this can't race.
        let period = self.period.fetch_add(1, Ordering::AcqRel) + 1;
        let t = (period as u64) << 15;

        critical_section::with(move |cs| {
            let n = 0;
            let alarm = &self.alarm.borrow(cs);
            let at = alarm.timestamp.get();

            if at < t + 0xc000 {
                // just enable it. `set_alarm` has already set the correct CCR val.
                timer.set_chn_ie(n + 1, true);
            }
        })
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut next = self
            .queue
            .borrow(cs)
            .borrow_mut()
            .next_expiration(self.now());

        while !self.set_alarm(cs, next) {
            next = self
                .queue
                .borrow(cs)
                .borrow_mut()
                .next_expiration(self.now());
        }
    }

    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        let timer = self.timer.try_get().unwrap().borrow(cs);

        let n = 0;
        self.alarm.borrow(cs).timestamp.set(timestamp);

        let t = self.now();
        if timestamp <= t {
            // If alarm timestamp has passed the alarm will not fire.
            // Disarm the alarm and return `false` to indicate that.
            timer.set_chn_ie(n + 1, false);

            self.alarm.borrow(cs).timestamp.set(u64::MAX);

            return false;
        }

        // Write the CCR value regardless of whether we're going to enable it now or not.
        // This way, when we enable it later, the right value is already set.
        timer.set_chn_cc_value(
            n + 1,
            timestamp as <RegisterBlock as ChannelsAccess>::Counter,
        );

        // Enable it if it'll happen soon. Otherwise, `next_period` will enable it.
        let diff = timestamp - t;
        timer.set_chn_ie(n + 1, diff < 0xc000);

        // Reevaluate if the alarm timestamp is still in the future
        let t = self.now();
        if timestamp <= t {
            // If alarm timestamp has passed since we set it, we have a race condition and
            // the alarm may or may not have fired.
            // Disarm the alarm and return `false` to indicate that.
            // It is the caller's responsibility to handle this ambiguity.
            timer.set_chn_ie(n + 1, false);

            self.alarm.borrow(cs).timestamp.set(u64::MAX);

            return false;
        }

        // We're confident the alarm will ring in the future.
        true
    }
}

impl Driver for EmbassyTimeDriver {
    fn now(&self) -> u64 {
        critical_section::with(|cs| {
            let timer = self.timer.try_get().unwrap().borrow(cs);
            let period = self.period.load(Ordering::Acquire);
            compiler_fence(Ordering::Acquire);
            // Timer1 has a 32-bit counter, while other timers resolution is limited to 16 bits
            let counter = timer.cnt().read().cnt().bits().into();
            calc_now(period, counter)
        })
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

time_driver_impl!(static DRIVER: EmbassyTimeDriver = EmbassyTimeDriver::new());

#[cfg(feature = "time-driver-tim1")]
#[interrupt]
fn TIMER1() {
    DRIVER.on_interrupt()
}

#[cfg(feature = "time-driver-tim2")]
#[interrupt]
fn TIMER2() {
    DRIVER.on_interrupt()
}

#[cfg(feature = "time-driver-tim14")]
#[interrupt]
fn TIMER14() {
    DRIVER.on_interrupt()
}
