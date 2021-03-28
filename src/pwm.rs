use crate::gpio::{
    gpioa::{PA10, PA11, PA8, PA9},
    Alternate, AF1,
};
use crate::pac::TIMER0;
use crate::time::Hertz;
use crate::time::U32Ext;
use crate::timer::{Timer, TimerExt};
use core::marker::Copy;

#[derive(Clone, Copy, Eq, PartialEq)]
pub enum Channel {
    C0,
    C1,
    C2,
    C3,
}

pub struct Pwm<TIMER, PINS>
where
    PINS: Pins<TIMER>,
{
    clock: Hertz,
    pins: PINS,
    timer: TIMER,
}

pub trait Pins<TIMER> {
    fn uses_channel(&self, channel: Channel) -> bool;
}

// TODO: Check which AF is correct
impl Pins<TIMER0>
    for (
        Option<PA8<Alternate<AF1>>>,
        Option<PA9<Alternate<AF1>>>,
        Option<PA10<Alternate<AF1>>>,
        Option<PA11<Alternate<AF1>>>,
    )
{
    fn uses_channel(&self, channel: Channel) -> bool {
        match channel {
            Channel::C0 => self.0.is_some(),
            Channel::C1 => self.1.is_some(),
            Channel::C2 => self.2.is_some(),
            Channel::C3 => self.3.is_some(),
        }
    }
}

impl Timer<TIMER0> {
    pub fn pwm<PINS, T>(self, pins: PINS, freq: T) -> Pwm<TIMER0, PINS>
    where
        PINS: Pins<TIMER0>,
        T: Into<Hertz>,
    {
        // TIMER0 has a break function that deactivates the outputs. This bit automatically activates
        // the output when no break input is present.
        self.timer.cchp.modify(|_, w| w.oaen().automatic());

        let Self { timer, clock } = self;
        timer0(timer, pins, freq.into(), clock)
    }
}

fn timer0<PINS>(mut timer: TIMER0, pins: PINS, freq: Hertz, clock: Hertz) -> Pwm<TIMER0, PINS>
where
    PINS: Pins<TIMER0>,
{
    if pins.uses_channel(Channel::C0) {
        timer
            .chctl0_output()
            .modify(|_, w| w.ch0comsen().set_bit().ch0comctl().pwm_mode1());
    }
    if pins.uses_channel(Channel::C1) {
        timer
            .chctl0_output()
            .modify(|_, w| w.ch1comsen().set_bit().ch1comctl().pwm_mode1());
    }
    if pins.uses_channel(Channel::C2) {
        timer
            .chctl1_output()
            .modify(|_, w| w.ch2comsen().set_bit().ch2comctl().pwm_mode1());
    }
    if pins.uses_channel(Channel::C3) {
        timer
            .chctl1_output()
            .modify(|_, w| w.ch3comsen().set_bit().ch3comctl().pwm_mode1());
    }
    timer.configure_prescaler_reload(freq, clock);
    // Trigger an update event to load the prescaler value to the clock
    timer.reset_counter();

    timer.ctl0.write(|w| {
        w.cam()
            .edge_aligned()
            .dir()
            .up()
            .spm()
            .disabled()
            .cen()
            .enabled()
    });

    Pwm {
        clock: clock,
        pins,
        timer,
    }
}

impl<PINS> embedded_hal::Pwm for Pwm<TIMER0, PINS>
where
    PINS: Pins<TIMER0>,
{
    type Channel = Channel;
    type Duty = u16;
    type Time = Hertz;

    fn enable(&mut self, channel: Self::Channel) {
        assert!(self.pins.uses_channel(channel));
        match channel {
            Channel::C0 => self.timer.chctl2.modify(|_, w| w.ch0en().enabled()),
            Channel::C1 => self.timer.chctl2.modify(|_, w| w.ch1en().enabled()),
            Channel::C2 => self.timer.chctl2.modify(|_, w| w.ch2en().enabled()),
            Channel::C3 => self.timer.chctl2.modify(|_, w| w.ch3en().enabled()),
        }
    }

    fn disable(&mut self, channel: Self::Channel) {
        assert!(self.pins.uses_channel(channel));
        match channel {
            Channel::C0 => self.timer.chctl2.modify(|_, w| w.ch0en().disabled()),
            Channel::C1 => self.timer.chctl2.modify(|_, w| w.ch1en().disabled()),
            Channel::C2 => self.timer.chctl2.modify(|_, w| w.ch2en().disabled()),
            Channel::C3 => self.timer.chctl2.modify(|_, w| w.ch3en().disabled()),
        }
    }

    fn get_duty(&self, channel: Self::Channel) -> Self::Duty {
        assert!(self.pins.uses_channel(channel));
        match channel {
            Channel::C0 => self.timer.ch0cv.read().ch0val().bits(),
            Channel::C1 => self.timer.ch1cv.read().ch1val().bits(),
            Channel::C2 => self.timer.ch2cv.read().ch2val().bits(),
            Channel::C3 => self.timer.ch3cv.read().ch3val().bits(),
        }
    }

    fn set_duty(&mut self, channel: Self::Channel, duty: Self::Duty) {
        assert!(self.pins.uses_channel(channel));
        match channel {
            Channel::C0 => self.timer.ch0cv.write(|w| w.ch0val().bits(duty)),
            Channel::C1 => self.timer.ch1cv.write(|w| w.ch1val().bits(duty)),
            Channel::C2 => self.timer.ch2cv.write(|w| w.ch2val().bits(duty)),
            Channel::C3 => self.timer.ch3cv.write(|w| w.ch3val().bits(duty)),
        }
    }

    fn get_max_duty(&self) -> Self::Duty {
        self.timer.car.read().car().bits()
    }

    fn get_period(&self) -> Self::Time {
        let presaler: u32 = self.timer.psc.read().psc().bits().into();
        let auto_reload_value: u32 = self.timer.car.read().car().bits().into();

        // Length in ms of an internal clock pulse
        (self.clock.0 / (presaler * auto_reload_value)).hz()
    }

    fn set_period<T>(&mut self, period: T)
    where
        T: Into<Self::Time>,
    {
        self.timer
            .configure_prescaler_reload(period.into(), self.clock);
    }
}
