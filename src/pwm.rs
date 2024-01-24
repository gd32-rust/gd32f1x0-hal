// Copyright 2021 The gd32f1x0-hal authors.
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use crate::gpio::{
    gpioa::{PA0, PA1, PA10, PA11, PA15, PA2, PA3, PA5, PA6, PA7, PA8, PA9},
    gpiob::{PB0, PB1, PB10, PB11, PB13, PB14, PB15, PB3, PB4, PB5},
    gpioc::{PC6, PC7, PC8, PC9},
    Alternate, AF0, AF1, AF2,
};
use crate::pac::{timer0, timer1, timer13, timer14, timer15, TIMER0, TIMER1, TIMER2};
use crate::time::Hertz;
use crate::time::U32Ext;
use crate::timer::{Event, Timer, TimerExt};
use core::convert::Infallible;
use core::marker::{Copy, PhantomData};
use core::ops::Deref;
use embedded_hal::pwm::{ErrorType, SetDutyCycle};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Channel {
    C0,
    C1,
    C2,
    C3,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Polarity {
    NotInverted,
    Inverted,
}

impl From<Polarity> for timer0::chctl2::CH0P_A {
    fn from(polarity: Polarity) -> Self {
        match polarity {
            Polarity::NotInverted => Self::NOT_INVERTED,
            Polarity::Inverted => Self::INVERTED,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IdleState {
    Low,
    High,
}

impl From<IdleState> for timer0::ctl1::ISO0_A {
    fn from(idle_state: IdleState) -> Self {
        match idle_state {
            IdleState::Low => Self::LOW,
            IdleState::High => Self::HIGH,
        }
    }
}

impl From<IdleState> for timer0::ctl1::ISO0N_A {
    fn from(idle_state: IdleState) -> Self {
        match idle_state {
            IdleState::Low => Self::LOW,
            IdleState::High => Self::HIGH,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BreakMode {
    Disabled,
    ActiveLow,
    ActiveHigh,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Alignment {
    /// Align the edge of the pulses from each channel.
    Edge,
    /// Align the center of the pulses from each channel.
    Center,
}

impl From<Alignment> for timer0::ctl0::CAM_A {
    fn from(alignment: Alignment) -> Self {
        match alignment {
            Alignment::Edge => Self::EDGE_ALIGNED,
            Alignment::Center => Self::CENTER_ALIGNED_COUNTING_UP,
        }
    }
}

#[doc(hidden)]
pub trait TimerRegExt {
    fn disable_channel(&self, channel: Channel, uses_complementary: bool);
    fn enable_channel(&self, channel: Channel, uses_complementary: bool);
    fn get_duty(&self, channel: Channel) -> u16;
    fn set_duty(&self, channel: Channel, duty: u16);
    fn get_max_duty(&self) -> u16;
    fn set_polarity(&self, channel: Channel, complementary: bool, polarity: Polarity);
}

#[doc(hidden)]
pub trait TimerIdleRegExt {
    fn set_idle_state(&self, channel: Channel, complementary: bool, idle_state: IdleState);
}

pub struct Pwm<TIMER, PINS> {
    clock: Hertz,
    pins: PINS,
    timer: TIMER,
}

/// A single channel of a PWM peripheral.
pub struct PwmChannel<TIMER, PIN> {
    channel: Channel,
    timer: *const dyn TimerRegExt,
    _pin: PIN,
    _timer: PhantomData<TIMER>,
}

/// A single channel of a PWM peripheral, including a complementary output.
pub struct PwmChannelComplementary<TIMER, PIN> {
    pwm_channel: PwmChannel<TIMER, PIN>,
}

pub struct Ch0;
pub struct Ch1;
pub struct Ch2;
pub struct Ch3;

pub trait Pin<TIMER, CHANNEL> {}
pub trait ComplementaryPin<TIMER, CHANNEL> {}

pub trait Pins<TIMER> {
    fn uses_channel(&self, channel: Channel) -> bool;
    fn uses_complementary_channel(&self, channel: Channel) -> bool;
}

/// Marker trait for pins which include complementary outputs.
pub trait ComplementaryPins {}

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

    fn uses_complementary_channel(&self, _channel: Channel) -> bool {
        false
    }
}

impl<P0, P0N, P1, P1N, P2, P2N, TIMER> Pins<TIMER>
    for (Option<(P0, P0N)>, Option<(P1, P1N)>, Option<(P2, P2N)>)
where
    P0: Pin<TIMER, Ch0>,
    P1: Pin<TIMER, Ch1>,
    P2: Pin<TIMER, Ch2>,
    P0N: ComplementaryPin<TIMER, Ch0>,
    P1N: ComplementaryPin<TIMER, Ch1>,
    P2N: ComplementaryPin<TIMER, Ch2>,
{
    fn uses_channel(&self, channel: Channel) -> bool {
        match channel {
            Channel::C0 => self.0.is_some(),
            Channel::C1 => self.1.is_some(),
            Channel::C2 => self.2.is_some(),
            Channel::C3 => false,
        }
    }

    fn uses_complementary_channel(&self, channel: Channel) -> bool {
        self.uses_channel(channel)
    }
}

impl<P0, P0N, P1, P1N, P2, P2N> ComplementaryPins
    for (Option<(P0, P0N)>, Option<(P1, P1N)>, Option<(P2, P2N)>)
{
}

impl<TIMER, PIN> PwmChannel<TIMER, PIN> {
    pub fn disable(&mut self) {
        unsafe { &*self.timer }.disable_channel(self.channel, false);
    }

    pub fn enable(&mut self) {
        unsafe { &*self.timer }.enable_channel(self.channel, false);
    }

    pub fn duty_cycle(&self) -> u16 {
        unsafe { &*self.timer }.get_duty(self.channel)
    }

    fn set_duty_cycle(&mut self, duty: u16) {
        unsafe { &*self.timer }.set_duty(self.channel, duty);
    }
}

impl<TIMER, PIN> ErrorType for PwmChannel<TIMER, PIN> {
    type Error = Infallible;
}

impl<TIMER, PIN> SetDutyCycle for PwmChannel<TIMER, PIN> {
    fn max_duty_cycle(&self) -> u16 {
        unsafe { &*self.timer }.get_max_duty()
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.set_duty_cycle(duty);
        Ok(())
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<TIMER, PIN> embedded_hal_02::PwmPin for PwmChannel<TIMER, PIN> {
    type Duty = u16;

    fn disable(&mut self) {
        self.disable();
    }

    fn enable(&mut self) {
        self.enable();
    }

    fn get_duty(&self) -> u16 {
        self.duty_cycle()
    }

    fn get_max_duty(&self) -> u16 {
        self.max_duty_cycle()
    }

    fn set_duty(&mut self, duty: u16) {
        self.set_duty_cycle(duty);
    }
}

impl<TIMER, PIN> PwmChannelComplementary<TIMER, PIN> {
    pub fn disable(&mut self) {
        unsafe { &*self.pwm_channel.timer }.disable_channel(self.pwm_channel.channel, true);
    }

    pub fn enable(&mut self) {
        unsafe { &*self.pwm_channel.timer }.enable_channel(self.pwm_channel.channel, true);
    }

    pub fn duty_cycle(&self) -> u16 {
        self.pwm_channel.duty_cycle()
    }

    fn set_duty_cycle(&mut self, duty: u16) {
        self.pwm_channel.set_duty_cycle(duty)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<TIMER, PIN> embedded_hal_02::PwmPin for PwmChannelComplementary<TIMER, PIN> {
    type Duty = u16;

    fn disable(&mut self) {
        self.disable();
    }

    fn enable(&mut self) {
        self.enable();
    }

    fn get_duty(&self) -> u16 {
        self.duty_cycle()
    }

    fn get_max_duty(&self) -> u16 {
        self.max_duty_cycle()
    }

    fn set_duty(&mut self, duty: u16) {
        self.set_duty_cycle(duty);
    }
}

impl<TIMER, PIN> ErrorType for PwmChannelComplementary<TIMER, PIN> {
    type Error = Infallible;
}

impl<TIMER, PIN> SetDutyCycle for PwmChannelComplementary<TIMER, PIN> {
    fn max_duty_cycle(&self) -> u16 {
        self.pwm_channel.max_duty_cycle()
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.set_duty_cycle(duty);
        Ok(())
    }
}

impl<TIMER: Deref<Target = RB>, RB: TimerIdleRegExt, PINS: Pins<TIMER>> Pwm<TIMER, PINS> {
    /// Configure the state which the output of the given channel should have when the channel is
    /// idle.
    pub fn set_idle_state(&self, channel: Channel, idle_state: IdleState) {
        assert!(self.pins.uses_channel(channel));
        self.timer.set_idle_state(channel, false, idle_state);
    }
}

impl<TIMER: Deref<Target = RB>, RB: TimerIdleRegExt, PINS: Pins<TIMER> + ComplementaryPins>
    Pwm<TIMER, PINS>
{
    /// Configure the state which the complementary output of the given channel should have when the
    /// channel is idle.
    pub fn set_complementary_idle_state(&self, channel: Channel, idle_state: IdleState) {
        assert!(self.pins.uses_complementary_channel(channel));
        self.timer.set_idle_state(channel, true, idle_state);
    }
}

impl<TIMER: Deref<Target = RB>, RB: TimerRegExt, PINS: Pins<TIMER>> Pwm<TIMER, PINS> {
    /// Configure the polarity of the output for the given channel.
    pub fn set_polarity(&self, channel: Channel, polarity: Polarity) {
        assert!(self.pins.uses_channel(channel));
        self.timer.set_polarity(channel, false, polarity);
    }
}

impl<TIMER: Deref<Target = RB>, RB: TimerRegExt, PINS: Pins<TIMER> + ComplementaryPins>
    Pwm<TIMER, PINS>
{
    /// Configure the polarity of the complementary output for the given channel.
    pub fn set_complementary_polarity(&self, channel: Channel, polarity: Polarity) {
        assert!(self.pins.uses_channel(channel));
        self.timer.set_polarity(channel, true, polarity);
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
                    self.timer.$cchp.modify(|_, w| w.oaen().automatic().prot().disabled());
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
                    .modify(|_, w| w.ch0comfen().slow().ch0comsen().enabled().ch0comctl().pwm_mode0());
            }
            if pins.uses_channel(Channel::C1) {
                timer
                    .chctl0_output()
                    .modify(|_, w| w.ch1comfen().slow().ch1comsen().enabled().ch1comctl().pwm_mode0());
            }
            if pins.uses_channel(Channel::C2) {
                timer
                    .chctl1_output()
                    .modify(|_, w| w.ch2comfen().slow().ch2comsen().enabled().ch2comctl().pwm_mode0());
            }
            if pins.uses_channel(Channel::C3) {
                timer
                    .chctl1_output()
                    .modify(|_, w| w.ch3comfen().slow().ch3comsen().enabled().ch3comctl().pwm_mode0());
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
                clock,
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

        impl<P0, P1, P2> Pwm<$TIMERX, (Option<P0>, Option<P1>, Option<P2>)> {
            /// Split the timer into separate PWM channels.
            pub fn split(
                self,
            ) -> (
                Option<PwmChannel<$TIMERX, P0>>,
                Option<PwmChannel<$TIMERX, P1>>,
                Option<PwmChannel<$TIMERX, P2>>,
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

            /// Configure the given alignment mode, to control how pulses on different channels of
            /// this PWM module are aligned with each other.
            pub fn set_alignment(&self, alignment: Alignment) {
                self.timer.ctl0.modify(|_, w| w.cam().variant(alignment.into()));
            }

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

            /// Returns true if the given `event` interrupt is pending.
            pub fn is_pending(&self, event: Event) -> bool {
                match event {
                    Event::Update => self.timer.intf.read().upif().is_update_pending(),
                }
            }

            /// Clears the given `event` interrupt flag.
            pub fn clear_interrupt_flag(&mut self, event: Event) {
                match event {
                    Event::Update => self.timer.intf.modify(|_, w| w.upif().clear()),
                }
            }


            pub fn disable(&mut self, channel: Channel) {
                assert!(self.pins.uses_channel(channel));
                self.timer.disable_channel(channel, self.pins.uses_complementary_channel(channel));
            }

            pub fn enable(&mut self, channel: Channel) {
                assert!(self.pins.uses_channel(channel));
                self.timer.enable_channel(channel, self.pins.uses_complementary_channel(channel));
            }

            pub fn duty_cycle(&self, channel: Channel) -> u16 {
                assert!(self.pins.uses_channel(channel));
                self.timer.get_duty(channel)
            }

            pub fn set_duty_cycle(&mut self, channel: Channel, duty: u16) {
                assert!(self.pins.uses_channel(channel));
                self.timer.set_duty(channel, duty);
            }

            pub fn max_duty_cycle(&self) -> u16 {
                self.timer.get_max_duty()
            }

            pub fn period(&self) -> Hertz {
                let presaler: u32 = self.timer.psc.read().psc().bits().into();
                let auto_reload_value: u32 = self.timer.car.read().car().bits().into();

                // Length in ms of an internal clock pulse
                (self.clock.0 / (presaler * auto_reload_value)).hz()
            }

            pub fn set_period<T>(&mut self, period: T)
            where
                T: Into<Hertz>,
            {
                self.timer
                    .configure_prescaler_reload(period.into(), self.clock);
                self.timer.reset_counter();
            }

            $(
                /// Disable PWM outputs, and prevent them from being automatically enabled.
                pub fn output_disable(&mut self) {
                    self.timer.$cchp.modify(|_, w| w.oaen().manual().poen().disabled());
                }

                /// Automatically enable outputs at the next update event, if the break input is not
                /// active.
                pub fn automatic_output_enable(&mut self) {
                    self.timer.$cchp.modify(|_, w| w.oaen().automatic());
                }

                /// Configure the given break mode.
                pub fn break_enable(&self, break_mode: BreakMode) {
                    match break_mode {
                        BreakMode::Disabled => self.timer.$cchp.modify(|_, w| w.brken().disabled()),
                        BreakMode::ActiveLow => self.timer.$cchp.modify(|_, w| w.brken().enabled().brkp().inverted()),
                        BreakMode::ActiveHigh => self.timer.$cchp.modify(|_, w| w.brken().enabled().brkp().not_inverted()),
                    }
                }

                /// Configure the run mode off-state.
                pub fn run_mode_off_state(&mut self, enabled: bool) {
                    self.timer.$cchp.modify(|_, w| w.ros().variant(
                        if enabled {
                            timer0::cchp::ROS_A::ENABLED
                        } else {
                            timer0::cchp::ROS_A::DISABLED
                        }
                    ));
                }

                /// Configure the idle mode off-state.
                pub fn idle_mode_off_state(&mut self, enabled: bool) {
                    self.timer.$cchp.modify(|_, w| w.ios().variant(
                        if enabled {
                            timer0::cchp::IOS_A::ENABLED
                        } else {
                            timer0::cchp::IOS_A::DISABLED
                        }
                    ));
                }

                /// Configure the dead time for complementary chanels.
                pub fn set_dead_time(&self, dead_time: u16) {
                    let dtcfg = if dead_time < 128 {
                        dead_time as u8
                    } else if dead_time < 256 {
                        0b1000_0000 | (dead_time / 2 - 64) as u8
                    } else if dead_time < 512 {
                        0b1100_0000 | (dead_time / 8 - 32) as u8
                    } else if dead_time < 1024 {
                        0b1110_0000 | (dead_time / 16 - 32) as u8
                    } else {
                        panic!("Invalid dead time {}", dead_time);
                    };
                    self.timer.$cchp.modify(|_, w| w.dtcfg().bits(dtcfg));
                }
            )?
        }

        #[cfg(feature = "embedded-hal-02")]
        impl<PINS> embedded_hal_02::Pwm for Pwm<$TIMERX, PINS>
        where
            PINS: Pins<$TIMERX>,
        {
            type Channel = Channel;
            type Duty = u16;
            type Time = Hertz;

            fn disable(&mut self, channel: Self::Channel) {
                self.disable(channel);
            }

            fn enable(&mut self, channel: Self::Channel) {
                self.enable(channel);
            }

            fn get_duty(&self, channel: Self::Channel) -> Self::Duty {
                self.duty_cycle(channel)
            }

            fn set_duty(&mut self, channel: Self::Channel, duty: Self::Duty) {
                self.set_duty_cycle(channel, duty);
            }

            fn get_max_duty(&self) -> Self::Duty {
                self.max_duty_cycle()
            }

            fn get_period(&self) -> Self::Time {
                self.period()
            }

            fn set_period<T>(&mut self, period: T)
            where
                T: Into<Self::Time>,
            {
                self.set_period(period);
            }
        }
    };
}

macro_rules! timer_reg_ext {
    ($timerX:ident, ($($channel:ident: $cv:ident, $val:ident, $p:ident $(/ $np:ident)?, $en:ident $(/ $nen:ident)? ;)+)) => {
        impl TimerRegExt for $timerX::RegisterBlock {
            fn disable_channel(&self, channel: Channel, uses_complementary: bool) {
                match channel {
                    $(
                        Channel::$channel => self.chctl2.modify(|_, w| w.$en().disabled()),
                    )+
                    #[allow(unreachable_patterns)]
                    _ => panic!("No such channel {:?}", channel),
                }
                if uses_complementary {
                    match channel {
                        $($(
                            Channel::$channel => self.chctl2.modify(|_, w| w.$nen().disabled()),
                        )?)+
                        _ => {}
                    }
                }
            }

            fn enable_channel(&self, channel: Channel, uses_complementary: bool) {
                match channel {
                    $(
                        Channel::$channel => self.chctl2.modify(|_, w| w.$en().enabled()),
                    )+
                    #[allow(unreachable_patterns)]
                    _ => panic!("No such channel {:?}", channel),
                }
                if uses_complementary {
                    match channel {
                        $($(
                            Channel::$channel => self.chctl2.modify(|_, w| w.$nen().enabled()),
                        )?)*
                        _ => {}
                    }
                }
            }

            fn get_duty(&self, channel: Channel) -> u16 {
                match channel {
                    $(
                        Channel::$channel => self.$cv.read().$val().bits() as u16,
                    )+
                    #[allow(unreachable_patterns)]
                    _ => panic!("No such channel {:?}", channel),
                }
            }

            fn set_duty(&self, channel: Channel, duty: u16) {
                let duty = duty.into();
                match channel {
                    $(
                        Channel::$channel => self.$cv.write(|w| w.$val().bits(duty)),
                    )+
                    #[allow(unreachable_patterns)]
                    _ => panic!("No such channel {:?}", channel),
                }
            }

            fn set_polarity(&self, channel: Channel, complementary: bool, polarity: Polarity) {
                match (channel, complementary) {
                    $(
                        (Channel::$channel, false) => {
                            self.chctl2.modify(|_, w| w.$p().variant(polarity.into()))
                        }
                        $(
                            (Channel::$channel, true) => {
                                self.chctl2.modify(|_, w| w.$np().variant(polarity.into()))
                            }
                        )?
                    )+
                    #[allow(unreachable_patterns)]
                    _ => panic!("No such channel {:?}/{}", channel, complementary),
                }
            }

            fn get_max_duty(&self) -> u16 {
                self.car.read().car().bits() as u16
            }
        }
    };
}

macro_rules! timer_idle_reg_ext {
    ($timerX:ident, ($($channel:ident: $iso:ident $(/ $ison:ident)? ;)+)) => {
        impl TimerIdleRegExt for $timerX::RegisterBlock {
            fn set_idle_state(&self, channel: Channel, complementary: bool, idle_state: IdleState) {
                match (channel, complementary) {
                    $(
                        (Channel::$channel, false) => {
                            self.ctl1.modify(|_, w| w.$iso().variant(idle_state.into()))
                        }
                        $(
                            (Channel::$channel, true) => {
                                self.ctl1.modify(|_, w| w.$ison().variant(idle_state.into()))
                            }
                            )?
                    )+
                    #[allow(unreachable_patterns)]
                    _ => panic!("No such channel {:?}/{}", channel, complementary),
                }
            }
        }
    };
}

impl Pin<TIMER0, Ch0> for PA8<Alternate<AF2>> {}
impl Pin<TIMER0, Ch1> for PA9<Alternate<AF2>> {}
impl Pin<TIMER0, Ch2> for PA10<Alternate<AF2>> {}
impl Pin<TIMER0, Ch3> for PA11<Alternate<AF2>> {}

impl ComplementaryPin<TIMER0, Ch0> for PB13<Alternate<AF2>> {}
impl ComplementaryPin<TIMER0, Ch1> for PB14<Alternate<AF2>> {}
impl ComplementaryPin<TIMER0, Ch2> for PB15<Alternate<AF2>> {}

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
timer_reg_ext!(timer0, (
    C0: ch0cv, ch0val, ch0p/ch0np, ch0en/ch0nen;
    C1: ch1cv, ch1val, ch1p/ch1np, ch1en/ch1nen;
    C2: ch2cv, ch2val, ch2p/ch2np, ch2en/ch2nen;
    C3: ch3cv, ch3val, ch3p, ch3en;
));
timer_idle_reg_ext!(timer0, (
    C0: iso0/iso0n;
    C1: iso1/iso1n;
    C2: iso2/iso2n;
    C3: iso3;
));
timer_reg_ext!(timer1, (
    C0: ch0cv, ch0val, ch0p/ch0np, ch0en;
    C1: ch1cv, ch1val, ch1p/ch1np, ch1en;
    C2: ch2cv, ch2val, ch2p/ch2np, ch2en;
    C3: ch3cv, ch3val, ch3p/ch3np, ch3en;
));
// TIMER13/15/16 only have 1 channel, and TIMER14 only has 2.
timer_reg_ext!(timer13, (
    C0: ch0cv, ch0val, ch0p/ch0np, ch0en;
));
timer_reg_ext!(timer14, (
    C0: ch0cv, ch0val, ch0p/ch0np, ch0en/ch0nen;
    C1: ch1cv, ch1val, ch1p/ch1np, ch1en;
));
timer_idle_reg_ext!(timer14, (
    C0: iso0/iso0n;
    C1: iso1;
));
timer_reg_ext!(timer15, (
    C0: ch0cv, ch0val, ch0p/ch0np, ch0en/ch0nen;
));
timer_idle_reg_ext!(timer15, (
    C0: iso0/iso0n;
));

hal!(TIMER0: (timer0, cchp));
hal!(TIMER1: (timer1));
hal!(TIMER2: (timer2));
