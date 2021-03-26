use crate::pac::rcu::AHBEN;
use crate::rcc::AHB;
use core::convert::Infallible;
use core::marker::PhantomData;
use embedded_hal::digital::v2::{toggleable, InputPin, OutputPin, StatefulOutputPin};

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, ahb: &mut AHBEN) -> Self::Parts;
}

/// Marker trait for active states.
pub trait Active {}

/// Input mode (type state)
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}
impl<MODE> Active for Input<MODE> {}

/// Used by the debugger (type state)
pub struct Debugger;
/// Floating input (type state)
pub struct Floating;
/// Pulled down input (type state)
pub struct PullDown;
/// Pulled up input (type state)
pub struct PullUp;

/// Output mode (type state)
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}
impl<MODE> Active for Output<MODE> {}

/// Push pull output (type state)
pub struct PushPull;
/// Open drain output (type state)
pub struct OpenDrain;

/// Analog mode (type state)
pub struct Analog;
impl Active for Analog {}

pub trait AF {
    const NUMBER: u32;
}

/// Alternate function 0
pub struct AF0;
impl AF for AF0 {
    const NUMBER: u32 = 0;
}

/// Alternate function 1
pub struct AF1;
/// Alternate function 2
pub struct AF2;
/// Alternate function 3
pub struct AF3;
/// Alternate function 4
pub struct AF4;
/// Alternate function 5
pub struct AF5;
/// Alternate function 6
pub struct AF6;
/// Alternate function 7
pub struct AF7;

/// Alternate function mode (type state)
pub struct Alternate<AFN> {
    _af: PhantomData<AFN>,
}
impl<AFN> Active for Alternate<AFN> {}

#[derive(Clone, Copy, Debug)]
pub enum Speed {
    Mhz2 = 0b00,
    Mhz10 = 0b01,
    Mhz50 = 0b11,
}

enum Mode {
    Input = 0b00,
    Output = 0b01,
    Alternate = 0b10,
    Analog = 0b11,
}

#[derive(Clone, Copy, Debug)]
pub enum PullMode {
    Floating = 0b00,
    PullUp = 0b01,
    PullDown = 0b10,
}

#[derive(Clone, Copy, Debug)]
pub enum OutputMode {
    PushPull = 0b0,
    OpenDrain = 0b1,
}

trait GpioRegExt {
    unsafe fn is_low(&self, pin_index: u8) -> bool;
    unsafe fn is_set_low(&self, pin_index: u8) -> bool;
    unsafe fn set_low(&self, pin_index: u8);
    unsafe fn set_high(&self, pin_index: u8);
}

// These impls are needed because a macro can not brace initialise a ty token
impl<MODE> Input<MODE> {
    const fn new() -> Self {
        Self { _mode: PhantomData }
    }
}
impl<MODE> Output<MODE> {
    const fn new() -> Self {
        Self { _mode: PhantomData }
    }
}
impl<AFN> Alternate<AFN> {
    const fn new() -> Self {
        Self { _af: PhantomData }
    }
}
impl Debugger {
    const fn new() -> Self {
        Self {}
    }
}

/// Erased GPIO pin on some port.
#[derive(Debug)]
pub struct Pin<MODE> {
    pin_index: u8,
    port: *const dyn GpioRegExt,
    _mode: MODE,
}

unsafe impl<MODE> Sync for Pin<MODE> {}
unsafe impl<MODE> Send for Pin<MODE> {}

impl<MODE> toggleable::Default for Pin<Output<MODE>> {}

impl<MODE> OutputPin for Pin<Output<MODE>> {
    type Error = Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        unsafe { Ok((*self.port).set_high(self.pin_index)) }
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        unsafe { Ok((*self.port).set_low(self.pin_index)) }
    }
}

impl<MODE> StatefulOutputPin for Pin<Output<MODE>> {
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        self.is_set_low().map(|b| !b)
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        unsafe { Ok((*self.port).is_set_low(self.pin_index)) }
    }
}

impl<MODE> InputPin for Pin<Input<MODE>> {
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        self.is_low().map(|b| !b)
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        unsafe { Ok((*self.port).is_low(self.pin_index)) }
    }
}

impl InputPin for Pin<Output<OpenDrain>> {
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        self.is_low().map(|b| !b)
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        unsafe { Ok((*self.port).is_low(self.pin_index)) }
    }
}

pub mod gpioa {
    use super::*;
    use crate::pac::{gpioa, rcu::AHBEN, GPIOA};

    pub struct Parts {
        pub config: Config,
        pub pa0: PA0<Input<Floating>>,
    }

    pub struct Config {
        _config: (),
    }

    impl GpioExt for GPIOA {
        type Parts = Parts;

        fn split(self, ahb: &mut AHBEN) -> Parts {
            ahb.modify(|_, w| w.paen().enabled());

            Parts {
                pa0: PA0 {
                    mode: <Input<Floating>>::new(),
                },
                config: Config { _config: () },
            }
        }
    }

    impl GpioRegExt for gpioa::RegisterBlock {
        unsafe fn is_low(&self, pin_index: u8) -> bool {
            self.istat.read().bits() & (1 << pin_index) == 0
        }

        unsafe fn is_set_low(&self, pin_index: u8) -> bool {
            self.octl.read().bits() & (1 << pin_index) == 0
        }

        unsafe fn set_low(&self, pin_index: u8) {
            self.bc.write(|w| w.bits(1 << pin_index))
        }

        unsafe fn set_high(&self, pin_index: u8) {
            self.bop.write(|w| w.bits(1 << pin_index))
        }
    }

    /// Configures the given pin to have the given alternate function mode
    unsafe fn set_alternate_mode(
        config: &mut Config,
        pin_index: u8,
        mode: u32,
        pull_mode: PullMode,
        output_mode: OutputMode,
    ) {
        let offset = (4 * pin_index) % 32;
        let reg = &(*GPIOA::ptr());
        if pin_index < 8 {
            reg.afsel0
                .modify(|r, w| w.bits((r.bits() & !(0b1111 << offset)) | (mode << offset)));
        } else {
            reg.afsel1
                .modify(|r, w| w.bits((r.bits() & !(0b1111 << offset)) | (mode << offset)));
        }
        set_mode(config, pin_index, Mode::Alternate, pull_mode, output_mode);
    }

    /// Sets the input/output, pull-up/down and output mode of a pin
    unsafe fn set_mode(
        _config: &mut Config,
        pin_index: u8,
        ctl: Mode,
        pud: PullMode,
        output_mode: OutputMode,
    ) {
        let offset = 2 * pin_index;
        let reg = &(*GPIOA::ptr());
        reg.pud
            .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | ((pud as u32) << offset)));
        reg.omode.modify(|r, w| {
            w.bits((r.bits() & !(0b1 << pin_index)) | ((output_mode as u32) << pin_index))
        });
        reg.ctl
            .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | ((ctl as u32) << offset)));
    }

    /// Sets the max slew rate of a pin.
    unsafe fn set_speed(_config: &mut Config, pin_index: u8, speed: Speed) {
        let offset = 2 * pin_index;
        let reg = &(*GPIOA::ptr());
        reg.ospd
            .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | ((speed as u32) << offset)));
    }

    pub struct PA0<MODE> {
        mode: MODE,
    }

    impl PA0<Debugger> {
        pub fn activate(self) -> PA0<Input<Floating>> {
            PA0 { mode: Input::new() }
        }
    }

    impl<MODE> PA0<MODE>
    where
        MODE: Active,
    {
        /// Configures the pin to operate as a floating input.
        #[inline]
        pub fn into_floating_input(self, config: &mut Config) -> PA0<Input<Floating>> {
            unsafe {
                set_mode(
                    config,
                    0,
                    Mode::Input,
                    PullMode::Floating,
                    OutputMode::PushPull,
                )
            };
            PA0 { mode: Input::new() }
        }

        /// Configures the pin to operate as a pulled down input.
        #[inline]
        pub fn into_pull_up_input(self, config: &mut Config) -> PA0<Input<PullUp>> {
            unsafe {
                set_mode(
                    config,
                    0,
                    Mode::Input,
                    PullMode::PullUp,
                    OutputMode::PushPull,
                )
            };
            PA0 { mode: Input::new() }
        }

        /// Configures the pin to operate as a pulled down input.
        #[inline]
        pub fn into_pull_down_input(self, config: &mut Config) -> PA0<Input<PullDown>> {
            unsafe {
                set_mode(
                    config,
                    0,
                    Mode::Input,
                    PullMode::PullDown,
                    OutputMode::PushPull,
                )
            };
            PA0 { mode: Input::new() }
        }

        /// Configures the pin to operate as an analog input or output.
        #[inline]
        pub fn into_analog(self, config: &mut Config) -> PA0<Analog> {
            unsafe {
                set_mode(
                    config,
                    0,
                    Mode::Analog,
                    PullMode::Floating,
                    OutputMode::PushPull,
                )
            };
            PA0 { mode: Analog {} }
        }

        /// Configures the pin to operate as an open drain output.
        #[inline]
        pub fn into_open_drain_output(self, config: &mut Config) -> PA0<Output<OpenDrain>> {
            unsafe {
                set_mode(
                    config,
                    0,
                    Mode::Output,
                    PullMode::Floating,
                    OutputMode::OpenDrain,
                )
            };
            PA0 {
                mode: Output::new(),
            }
        }

        /// Configures the pin to operate as an open drain output.
        #[inline]
        pub fn into_push_pull_output(self, config: &mut Config) -> PA0<Output<PushPull>> {
            unsafe {
                set_mode(
                    config,
                    0,
                    Mode::Output,
                    PullMode::Floating,
                    OutputMode::PushPull,
                )
            };
            PA0 {
                mode: Output::new(),
            }
        }

        /// Configures the pin to operate as an alternate function.
        pub fn into_alternate<AFN: AF>(
            self,
            config: &mut Config,
            pull_mode: PullMode,
            output_mode: OutputMode,
        ) -> PA0<Alternate<AFN>> {
            unsafe { set_alternate_mode(config, 0, AFN::NUMBER, pull_mode, output_mode) };
            PA0 {
                mode: Alternate::new(),
            }
        }

        /// Erases the pin number and port from the type.
        pub fn downgrade(self) -> Pin<MODE> {
            Pin {
                pin_index: 0,
                port: GPIOA::ptr(),
                _mode: self.mode,
            }
        }
    }

    impl<MODE> PA0<Output<MODE>> {
        /// Configures the max slew rate of the pin
        pub fn set_speed(&mut self, config: &mut Config, speed: Speed) {
            unsafe { set_speed(config, 0, speed) };
        }
    }

    impl<AFN> PA0<Alternate<AFN>> {
        /// Configures the max slew rate of the pin
        pub fn set_speed(&mut self, config: &mut Config, speed: Speed) {
            unsafe { set_speed(config, 0, speed) };
        }
    }

    impl<MODE> toggleable::Default for PA0<Output<MODE>> {}

    impl<MODE> OutputPin for PA0<Output<MODE>> {
        type Error = Infallible;

        fn set_high(&mut self) -> Result<(), Self::Error> {
            Ok(unsafe { (*GPIOA::ptr()).set_high(0) })
        }

        fn set_low(&mut self) -> Result<(), Self::Error> {
            Ok(unsafe { (*GPIOA::ptr()).set_low(0) })
        }
    }

    impl<MODE> StatefulOutputPin for PA0<Output<MODE>> {
        fn is_set_high(&self) -> Result<bool, Self::Error> {
            self.is_set_low().map(|b| !b)
        }

        fn is_set_low(&self) -> Result<bool, Self::Error> {
            Ok(unsafe { (*GPIOA::ptr()).is_set_low(0) })
        }
    }

    impl<MODE> InputPin for PA0<Input<MODE>> {
        type Error = Infallible;

        fn is_high(&self) -> Result<bool, Self::Error> {
            self.is_low().map(|b| !b)
        }

        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(unsafe { (*GPIOA::ptr()).is_low(0) })
        }
    }

    impl InputPin for PA0<Output<OpenDrain>> {
        type Error = Infallible;

        fn is_high(&self) -> Result<bool, Self::Error> {
            self.is_low().map(|b| !b)
        }

        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(unsafe { (*GPIOA::ptr()).is_low(0) })
        }
    }
}
