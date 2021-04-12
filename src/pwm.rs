// Copyright 2021 The gd32f1x0-hal authors.
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use crate::gpio::{
    gpioa::{PA0, PA1, PA10, PA11, PA15, PA2, PA3, PA5, PA6, PA7, PA8, PA9},
    gpiob::{PB0, PB1, PB10, PB11, PB3, PB4, PB5},
    gpioc::{PC6, PC7, PC8, PC9},
    Alternate, AF0, AF1, AF2,
};
use crate::pac::{timer0, timer1, TIMER0, TIMER1, TIMER2};
use crate::time::Hertz;
use crate::time::U32Ext;
use crate::timer::{Timer, TimerExt};
use core::marker::{Copy, PhantomData};

#[derive(Clone, Copy, Eq, PartialEq)]
pub enum Channel {
    C0,
    C1,
    C2,
    C3,
}

trait TimerRegExt {
    fn disable_channel(&self, channel: Channel);
    fn enable_channel(&self, channel: Channel);
    fn get_duty(&self, channel: Channel) -> u16;
    fn set_duty(&self, channel: Channel, duty: u16);
    fn get_max_duty(&self) -> u16;
}

pub struct Pwm<TIMER, PINS> {
    clock: Hertz,
    pins: PINS,
    timer: TIMER,
}

pub struct PwmChannel<TIMER, PIN> {
    channel: Channel,
    timer: *const dyn TimerRegExt,
    _pin: PIN,
    _timer: PhantomData<TIMER>,
}

pub struct Ch0;
pub struct Ch1;
pub struct Ch2;
pub struct Ch3;

pub trait Pin<TIMER, CHANNEL> {}

pub trait Pins<TIMER> {
    fn uses_channel(&self, channel: Channel) -> bool;
}

impl<P0, P1, P2, P3, TIMER> Pins<TIMER> for (Option<P0>, Option<P1>, Option<P2>, Option<P3>)
where
    P0: Pin<TIMER, Ch0>,
    P1: Pin<TIMER, Ch1>,
    P2: Pin<TIMER, Ch2>,
    P3: Pin<TIMER, Ch3>,
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

impl<TIMER, PIN> embedded_hal::PwmPin for PwmChannel<TIMER, PIN> {
    type Duty = u16;

    fn disable(&mut self) {
        unsafe { &*self.timer }.disable_channel(self.channel);
    }

    fn enable(&mut self) {
        unsafe { &*self.timer }.enable_channel(self.channel);
    }

    fn get_duty(&self) -> u16 {
        unsafe { &*self.timer }.get_duty(self.channel)
    }

    fn get_max_duty(&self) -> u16 {
        unsafe { &*self.timer }.get_max_duty()
    }

    fn set_duty(&mut self, duty: u16) {
        unsafe { &*self.timer }.set_duty(self.channel, duty);
    }
}

macro_rules! hal {
    ($TIMERX:ident: ($timerX:ident $(,$cchp:ident)*)) => {
        impl Timer<$TIMERX> {
            pub fn pwm<PINS, T>(self, pins: PINS, freq: T) -> Pwm<$TIMERX, PINS>
            where
                PINS: Pins<$TIMERX>,
                T: Into<Hertz>,
            {
                $(
                    // Some timers have a break function that deactivates the outputs. This bit
                    // automatically activates the output when no break input is present.
                    self.timer.$cchp.modify(|_, w| w.oaen().automatic());
                )?

                let Self { timer, clock } = self;
                $timerX(timer, pins, freq.into(), clock)
            }
        }

        fn $timerX<PINS>(
            mut timer: $TIMERX,
            pins: PINS,
            freq: Hertz,
            clock: Hertz,
        ) -> Pwm<$TIMERX, PINS>
        where
            PINS: Pins<$TIMERX>,
        {
            if pins.uses_channel(Channel::C0) {
                timer
                    .chctl0_output()
                    .modify(|_, w| w.ch0comsen().enabled().ch0comctl().pwm_mode1());
            }
            if pins.uses_channel(Channel::C1) {
                timer
                    .chctl0_output()
                    .modify(|_, w| w.ch1comsen().enabled().ch1comctl().pwm_mode1());
            }
            if pins.uses_channel(Channel::C2) {
                timer
                    .chctl1_output()
                    .modify(|_, w| w.ch2comsen().enabled().ch2comctl().pwm_mode1());
            }
            if pins.uses_channel(Channel::C3) {
                timer
                    .chctl1_output()
                    .modify(|_, w| w.ch3comsen().enabled().ch3comctl().pwm_mode1());
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

        impl<P0, P1, P2, P3> Pwm<$TIMERX, (Option<P0>, Option<P1>, Option<P2>, Option<P3>)> {
            /// Split the timer into separate PWM channels.
            pub fn split(
                self,
            ) -> (
                Option<PwmChannel<$TIMERX, P0>>,
                Option<PwmChannel<$TIMERX, P1>>,
                Option<PwmChannel<$TIMERX, P2>>,
                Option<PwmChannel<$TIMERX, P3>>,
            ) {
                (
                    self.pins.0.map(|pin| PwmChannel {
                        channel: Channel::C0,
                        timer: $TIMERX::ptr(),
                        _pin: pin,
                        _timer: PhantomData,
                    }),
                    self.pins.1.map(|pin| PwmChannel {
                        channel: Channel::C1,
                        timer: $TIMERX::ptr(),
                        _pin: pin,
                        _timer: PhantomData,
                    }),
                    self.pins.2.map(|pin| PwmChannel {
                        channel: Channel::C2,
                        timer: $TIMERX::ptr(),
                        _pin: pin,
                        _timer: PhantomData,
                    }),
                    self.pins.3.map(|pin| PwmChannel {
                        channel: Channel::C3,
                        timer: $TIMERX::ptr(),
                        _pin: pin,
                        _timer: PhantomData,
                    }),
                )
            }
        }

        impl<PINS> Pwm<$TIMERX, PINS>
        where
            PINS: Pins<$TIMERX>,
        {
            /// Stop the timer and release it and the pins to be used for something else.
            pub fn stop(self) -> (Timer<$TIMERX>, PINS) {
                self.timer.chctl2.reset();
                self.timer.chctl0_output().reset();
                self.timer.chctl1_output().reset();
                self.timer.ch0cv.reset();
                self.timer.ch1cv.reset();
                self.timer.ch2cv.reset();
                self.timer.ch3cv.reset();
                $(
                    self.timer.$cchp.reset();
                )?
                (
                    Timer {
                        timer: self.timer,
                        clock: self.clock,
                    },
                    self.pins,
                )
            }
        }

        impl<PINS> embedded_hal::Pwm for Pwm<$TIMERX, PINS>
        where
            PINS: Pins<$TIMERX>,
        {
            type Channel = Channel;
            type Duty = u16;
            type Time = Hertz;

            fn disable(&mut self, channel: Self::Channel) {
                assert!(self.pins.uses_channel(channel));
                self.timer.disable_channel(channel);
            }

            fn enable(&mut self, channel: Self::Channel) {
                assert!(self.pins.uses_channel(channel));
                self.timer.enable_channel(channel);
            }

            fn get_duty(&self, channel: Self::Channel) -> Self::Duty {
                assert!(self.pins.uses_channel(channel));
                self.timer.get_duty(channel)
            }

            fn set_duty(&mut self, channel: Self::Channel, duty: Self::Duty) {
                assert!(self.pins.uses_channel(channel));
                self.timer.set_duty(channel, duty);
            }

            fn get_max_duty(&self) -> Self::Duty {
                self.timer.get_max_duty()
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
    };
}

macro_rules! timer_reg_ext {
    ($timerX:ident) => {
        impl TimerRegExt for $timerX::RegisterBlock {
            fn disable_channel(&self, channel: Channel) {
                match channel {
                    Channel::C0 => self.chctl2.modify(|_, w| w.ch0en().disabled()),
                    Channel::C1 => self.chctl2.modify(|_, w| w.ch1en().disabled()),
                    Channel::C2 => self.chctl2.modify(|_, w| w.ch2en().disabled()),
                    Channel::C3 => self.chctl2.modify(|_, w| w.ch3en().disabled()),
                }
            }

            fn enable_channel(&self, channel: Channel) {
                match channel {
                    Channel::C0 => self.chctl2.modify(|_, w| w.ch0en().enabled()),
                    Channel::C1 => self.chctl2.modify(|_, w| w.ch1en().enabled()),
                    Channel::C2 => self.chctl2.modify(|_, w| w.ch2en().enabled()),
                    Channel::C3 => self.chctl2.modify(|_, w| w.ch3en().enabled()),
                }
            }

            fn get_duty(&self, channel: Channel) -> u16 {
                match channel {
                    Channel::C0 => self.ch0cv.read().ch0val().bits() as u16,
                    Channel::C1 => self.ch1cv.read().ch1val().bits() as u16,
                    Channel::C2 => self.ch2cv.read().ch2val().bits() as u16,
                    Channel::C3 => self.ch3cv.read().ch3val().bits() as u16,
                }
            }

            fn set_duty(&self, channel: Channel, duty: u16) {
                let duty = duty.into();
                match channel {
                    Channel::C0 => self.ch0cv.write(|w| w.ch0val().bits(duty)),
                    Channel::C1 => self.ch1cv.write(|w| w.ch1val().bits(duty)),
                    Channel::C2 => self.ch2cv.write(|w| w.ch2val().bits(duty)),
                    Channel::C3 => self.ch3cv.write(|w| w.ch3val().bits(duty)),
                }
            }

            fn get_max_duty(&self) -> u16 {
                self.car.read().car().bits() as u16
            }
        }
    };
}

impl Pin<TIMER0, Ch0> for PA8<Alternate<AF2>> {}
impl Pin<TIMER0, Ch1> for PA9<Alternate<AF2>> {}
impl Pin<TIMER0, Ch2> for PA10<Alternate<AF2>> {}
impl Pin<TIMER0, Ch3> for PA11<Alternate<AF2>> {}

impl Pin<TIMER1, Ch0> for PA0<Alternate<AF2>> {}
impl Pin<TIMER1, Ch0> for PA5<Alternate<AF2>> {}
impl Pin<TIMER1, Ch0> for PA15<Alternate<AF2>> {}
impl Pin<TIMER1, Ch1> for PA1<Alternate<AF2>> {}
impl Pin<TIMER1, Ch1> for PB3<Alternate<AF2>> {}
impl Pin<TIMER1, Ch2> for PA2<Alternate<AF2>> {}
impl Pin<TIMER1, Ch2> for PB10<Alternate<AF2>> {}
impl Pin<TIMER1, Ch3> for PA3<Alternate<AF2>> {}
impl Pin<TIMER1, Ch3> for PB11<Alternate<AF2>> {}

impl Pin<TIMER2, Ch0> for PA6<Alternate<AF1>> {}
impl Pin<TIMER2, Ch0> for PB4<Alternate<AF1>> {}
impl Pin<TIMER2, Ch0> for PC6<Alternate<AF0>> {}
impl Pin<TIMER2, Ch1> for PA7<Alternate<AF1>> {}
impl Pin<TIMER2, Ch1> for PB5<Alternate<AF1>> {}
impl Pin<TIMER2, Ch1> for PC7<Alternate<AF0>> {}
impl Pin<TIMER2, Ch2> for PB0<Alternate<AF1>> {}
impl Pin<TIMER2, Ch2> for PC8<Alternate<AF0>> {}
impl Pin<TIMER2, Ch3> for PB1<Alternate<AF1>> {}
impl Pin<TIMER2, Ch3> for PC9<Alternate<AF0>> {}

// Some timers share the same PAC types so we don't need this for all of them.
timer_reg_ext!(timer0);
timer_reg_ext!(timer1);
// TIMER13/15/16 only has 1 channel, and TIMER14 only has 2.
//timer_reg_ext!(timer13);
//timer_reg_ext!(timer14);
//timer_reg_ext!(timer15);

hal!(TIMER0: (timer0, cchp));
hal!(TIMER1: (timer1));
hal!(TIMER2: (timer2));
