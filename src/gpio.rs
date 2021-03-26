//! # General Purpose I/Os
//!
//! The GPIO pins are organised into groups of 16 pins which can be accessed through the
//! `gpioa`, `gpiob`... modules. To get access to the pins, you first need to convert them into a
//! HAL designed struct from the `pac` struct using the [split](trait.GpioExt.html#tymethod.split) function.
//! ```rust
//! // Acquire the GPIOC peripheral
//! // NOTE: `dp` is the device peripherals from the `PAC` crate
//! let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
//! ```
//!
//! See the documentation for [rcc::APB2](../rcc/struct.APB2.html) for details about the input parameter to
//! `split`.
//!
//! This gives you a struct containing two control registers `crl` and `crh`, and all the pins
//! `px0..px15`. These structs are what you use to interract with the pins to change their modes,
//! or their inputs or outputs. For example, to set `pa5` high, you would call
//!
//! ```rust
//! let output = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);
//! output.set_high();
//! ```
//!
//! Each GPIO pin can be set to various modes:
//!
//! - **Alternate**: Pin mode required when the pin is driven by other peripherals
//! - **Dynamic**: Pin mode is selected at runtime. See changing configurations for more details
//! - Input
//!     - **PullUp**: Input connected to high with a weak pull up resistor. Will be high when nothing
//!     is connected
//!     - **PullDown**: Input connected to high with a weak pull up resistor. Will be low when nothing
//!     is connected
//!     - **Floating**: Input not pulled to high or low. Will be undefined when nothing is connected
//! - Output
//!     - **PushPull**: Output which either drives the pin high or low
//!     - **OpenDrain**: Output which leaves the gate floating, or pulls it do ground in drain
//!     mode. Can be used as an input in the `open` configuration
//! - **Debugger**: Some pins start out being used by the debugger. A pin in this mode can only be
//! used if the [JTAG peripheral has been turned off](#accessing-pa15-pb3-and-pb14).
//!
//! ## Changing modes
//! The simplest way to change the pin mode is to use the `into_<mode>` functions. These return a
//! new struct with the correct mode that you can use the input or output functions on.
//!
//! If you need a more temporary mode change, and can not use the `into_<mode>` functions for
//! ownership reasons, you can use the `as_<mode>` functions to temporarily change the pin type, do
//! some output or input, and then have it change back once done.
//!
//! ### Dynamic Mode Change
//! The above mode change methods guarantee that you can only call input functions when the pin is
//! in input mode, and output when in output modes, but can lead to some issues. Therefore, there
//! is also a mode where the state is kept track of at runtime, allowing you to change the mode
//! often, and without problems with ownership, or references, at the cost of some performance and
//! the risk of runtime errors.
//!
//! To make a pin dynamic, use the `into_dynamic` function, and then use the `make_<mode>` functions to
//! change the mode
//!
//! ## Accessing PA15, PB3, and PB14
//!
//! These pins are used by the JTAG peripheral by default. To use them in your program, you need to
//! disable that peripheral. This is done using the [afio::MAPR::disable_jtag](../afio/struct.MAPR.html#method.disable_jtag) function
//!
//! # Interfacing with v1 traits
//!
//! `embedded-hal` has two versions of the digital traits, `v2` which is used by this crate and
//! `v1` which is deprecated but still used by a lot of drivers.  If you want to use such a driver
//! with this crate, you need to convert the digital pins to the `v1` type.
//!
//! This is done using `embedded-hal::digital::v1_compat::OldOutputPin`. For example:
//!
//! ```rust
//! let nss = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
//! let mut mfrc522 = Mfrc522::new(spi, OldOutputPin::from(nss)).unwrap();
//! ```

use core::marker::PhantomData;

use crate::pac::EXTI;
use crate::rcc::AHB;

/// Slew rates available for Output and relevant AlternateMode Pins
///
/// See Table 21 "Output MODE bits" in the reference
pub enum IOPinSpeed {
    /// Slew at 10Mhz
    Mhz10 = 0b01, // (yes, this one is "less" then 2Mhz)
    /// Slew at 2Mhz
    Mhz2 = 0b10,
    /// Slew at 50Mhz
    Mhz50 = 0b11,
}

/// Allow setting of the slew rate of an IO pin
///
/// Initially all pins are set to the maximum slew rate
pub trait OutputSpeed<CR> {
    fn set_speed(&mut self, cr: &mut CR, speed: IOPinSpeed);
}

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, ahb: &mut AHB) -> Self::Parts;
}

/// Marker trait for pin mode detection.
pub trait Mode<MODE> {}

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

/// Alternate function
pub struct Alternate<MODE> {
    _mode: PhantomData<MODE>,
}
impl<MODE> Active for Alternate<MODE> {}

pub enum State {
    High,
    Low,
}

/// NOTE: This trait should ideally be private but must be pub in order to avoid
/// complaints from the compiler.
pub trait PinMode<CR> {
    unsafe fn set_mode(cr: &mut CR) -> Self;
}

// These impls are needed because a macro can not brace initialise a ty token
impl<MODE> Input<MODE> {
    const fn _new() -> Self {
        Self { _mode: PhantomData }
    }
}
impl<MODE> Output<MODE> {
    const fn _new() -> Self {
        Self { _mode: PhantomData }
    }
}
impl<MODE> Alternate<MODE> {
    const fn _new() -> Self {
        Self { _mode: PhantomData }
    }
}
impl Debugger {
    const fn _new() -> Self {
        Self {}
    }
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $pen:ident, $PXx:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use core::convert::Infallible;
            use core::marker::PhantomData;

            use crate::hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, toggleable};
            use crate::pac::{$GPIOX, rcu::AHBEN};

            use crate::rcc::{AHB, Enable};
            use super::{
                Alternate, Floating, GpioExt, Input,
                OpenDrain,
                Output,
                PullDown,
                PullUp,
                PushPull,
                Analog,
                State,
                Active,
                Debugger,
                Pxx,
                Mode,
                PinMode,
                OutputSpeed,
                IOPinSpeed,
            };

            /// GPIO parts
            pub struct Parts {
                // TODO: control registers?
                $(
                    /// Pin
                    pub $pxi: $PXi<$MODE>,
                )+
            }

            impl GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self, ahb: &mut AHBEN) -> Parts {
                    ahb.modify(|_, w| w.$pen().enabled());
                    // TODO: Reset?

                    Parts {
                        $(
                            $pxi: $PXi { mode: <$MODE>::_new() },
                        )+
                    }
                }
            }

            /// Configure the given pin to have the given alternate function mode
            fn set_alternate_mode(index: usize, mode: u32) {
                let offset = 2 * index;
                let offset2 = 4 * index;
                let reg = unsafe { &(*$GPIOX::ptr())};
                if offset2 < 32 {
                    reg.afsel0.modify(|r, w| {
                        w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                    });
                } else {
                    let offset2 = offset2 - 32;
                    reg.afsel1.modify(|r, w| {
                        w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                    });
                }
                // Set alternate function mode for pin (0b10)
                reg.ctl.modify(|r, w| {
                    w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset))
                });
            }

            $(
                /// Pin
                pub struct $PXi<MODE> {
                    mode: MODE,
                }

                impl<MODE> Mode<MODE> for $PXi<MODE> {}

                impl $PXi<Debugger> {
                    /// Put the pin in an active state. The caller
                    /// must enforce that the pin is really in this
                    /// state in the hardware.
                    #[allow(dead_code)]
                    pub(crate) unsafe fn activate(self) -> $PXi<Input<Floating>> {
                        $PXi { mode: Input::_new() }
                    }
                }

                impl<MODE> $PXi<MODE> where MODE: Active {
                    /// Configures the pin to operate as an alternate function push-pull output
                    /// pin.
                    #[inline]
                    pub fn into_alternate_push_pull(
                        self,
                    ) -> $PXi<Alternate<PushPull>> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Alternate function output push pull
                        const CNF: u32 = 0b10;
                        // Output mode, max speed 50 MHz
                        const MODE: u32 = 0b11;
                        const BITS: u32 = (CNF << 2) | MODE;

                        // input mode
                        cr
                            .cr()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { mode: Alternate::_new() }
                    }

                    /// Configures the pin to operate as an alternate function open-drain output
                    /// pin.
                    #[inline]
                    pub fn into_alternate_open_drain(
                        self,
                    ) -> $PXi<Alternate<OpenDrain>> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Alternate function output open drain
                        const CNF: u32 = 0b11;
                        // Output mode, max speed 50 MHz
                        const MODE: u32 = 0b11;
                        const BITS: u32 = (CNF << 2) | MODE;

                        // input mode
                        cr
                            .cr()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { mode: Alternate::_new() }
                    }

                    /// Configures the pin to operate as a floating input pin
                    #[inline]
                    pub fn into_floating_input(
                        self,
                    ) -> $PXi<Input<Floating>> {
                        unsafe {
                            $PXi::<Input<Floating>>::set_mode(cr)
                        }
                    }

                    /// Configures the pin to operate as a pulled down input pin
                    #[inline]
                    pub fn into_pull_down_input(
                        self,
                    ) -> $PXi<Input<PullDown>> {
                        unsafe {
                            $PXi::<Input<PullDown>>::set_mode(cr)
                        }
                    }

                    /// Configures the pin to operate as a pulled up input pin
                    #[inline]
                    pub fn into_pull_up_input(
                        self,
                    ) -> $PXi<Input<PullUp>> {
                        unsafe {
                            $PXi::<Input<PullUp>>::set_mode(cr)
                        }
                    }

                    /// Configures the pin to operate as an open-drain output pin.
                    /// Initial state will be low.
                    #[inline]
                    pub fn into_open_drain_output(
                        self,
                    ) -> $PXi<Output<OpenDrain>> {
                        self.into_open_drain_output_with_state(cr, State::Low)
                    }

                    /// Configures the pin to operate as an open-drain output pin.
                    /// `initial_state` specifies whether the pin should be initially high or low.
                    #[inline]
                    pub fn into_open_drain_output_with_state(
                        mut self,
                        initial_state: State,
                    ) -> $PXi<Output<OpenDrain>> {
                        self.set_state(initial_state);
                        unsafe {
                            $PXi::<Output<OpenDrain>>::set_mode(cr)
                        }
                    }
                    /// Configures the pin to operate as an push-pull output pin.
                    /// Initial state will be low.
                    #[inline]
                    pub fn into_push_pull_output(
                        self,
                    ) -> $PXi<Output<PushPull>> {
                        self.into_push_pull_output_with_state(cr, State::Low)
                    }

                    /// Configures the pin to operate as an push-pull output pin.
                    /// `initial_state` specifies whether the pin should be initially high or low.
                    #[inline]
                    pub fn into_push_pull_output_with_state(
                        mut self,
                        initial_state: State,
                    ) -> $PXi<Output<PushPull>> {
                        self.set_state(initial_state);
                        unsafe {
                            $PXi::<Output<PushPull>>::set_mode(cr)
                        }
                    }

                    /// Configures the pin to operate as an analog input pin
                    #[inline]
                    pub fn into_analog(self) -> $PXi<Analog> {
                        unsafe {
                            $PXi::<Analog>::set_mode(cr)
                        }
                    }
                }

                // These macros are defined here instead of at the top level in order
                // to be able to refer to macro variables from the outer layers.
                macro_rules! impl_temp_output {
                    (
                        $fn_name:ident,
                        $stateful_fn_name:ident,
                        $mode:ty
                    ) => {
                        /**
                          Temporarily change the mode of the pin.

                          The value of the pin after conversion is undefined. If you
                          want to control it, use `$stateful_fn_name`
                        */
                        #[inline]
                        pub fn $fn_name(
                            &mut self,
                            mut f: impl FnMut(&mut $PXi<$mode>)
                        ) {
                            let mut temp = unsafe { $PXi::<$mode>::set_mode(cr) };
                            f(&mut temp);
                            unsafe {
                                Self::set_mode(cr);
                            }
                        }

                        /**
                          Temporarily change the mode of the pin.

                          Note that the new state is set slightly before conversion
                          happens. This can cause a short output glitch if switching
                          between output modes
                        */
                        #[inline]
                        pub fn $stateful_fn_name(
                            &mut self,
                            state: State,
                            mut f: impl FnMut(&mut $PXi<$mode>)
                        ) {
                            self.set_state(state);
                            let mut temp = unsafe { $PXi::<$mode>::set_mode(cr) };
                            f(&mut temp);
                            unsafe {
                                Self::set_mode(cr);
                            }
                        }
                    }
                }
                macro_rules! impl_temp_input {
                    (
                        $fn_name:ident,
                        $mode:ty
                    ) => {
                        /**
                          Temporarily change the mode of the pin.
                        */
                        #[inline]
                        pub fn $fn_name(
                            &mut self,
                            mut f: impl FnMut(&mut $PXi<$mode>)
                        ) {
                            let mut temp = unsafe { $PXi::<$mode>::set_mode(cr) };
                            f(&mut temp);
                            unsafe {
                                Self::set_mode(cr);
                            }
                        }
                    }
                }

                impl<MODE> $PXi<MODE> where MODE: Active, $PXi<MODE>: PinMode {
                    impl_temp_output!(
                        as_push_pull_output,
                        as_push_pull_output_with_state,
                        Output<PushPull>
                    );
                    impl_temp_output!(
                        as_open_drain_output,
                        as_open_drain_output_with_state,
                        Output<OpenDrain>
                    );
                    impl_temp_input!(
                        as_floating_input,
                        Input<Floating>
                    );
                    impl_temp_input!(
                        as_pull_up_input,
                        Input<PullUp>
                    );
                    impl_temp_input!(
                        as_pull_down_input,
                        Input<PullDown>
                    );
                }

                impl<MODE> $PXi<MODE> where MODE: Active {
                    /// Erases the pin number from the type
                    #[inline]
                    fn into_generic(self) -> Generic<MODE> {
                        Generic {
                            i: $i,
                            _mode: PhantomData,
                        }
                    }

                    /// Erases the pin number and port from the type
                    ///
                    /// This is useful when you want to collect the pins into an array where you
                    /// need all the elements to have the same type
                    pub fn downgrade(self) -> Pxx<MODE> {
                        self.into_generic().downgrade()
                    }
                }

                // embedded_hal impls

                impl<MODE> OutputPin for $PXi<Output<MODE>> {
                    type Error = Infallible;
                    #[inline]
                    fn set_high(&mut self) -> Result<(), Self::Error> {
                        // NOTE(unsafe) atomic write to a stateless register
                        Ok(self.set_state(State::High))
                    }

                    #[inline]
                    fn set_low(&mut self) -> Result<(), Self::Error> {
                        // NOTE(unsafe) atomic write to a stateless register
                        Ok(self.set_state(State::Low))
                    }
                }

                impl<MODE> StatefulOutputPin for $PXi<Output<MODE>> {
                    #[inline]
                    fn is_set_high(&self) -> Result<bool, Self::Error> {
                        self.is_set_low().map(|b| !b)
                    }

                    #[inline]
                    fn is_set_low(&self) -> Result<bool, Self::Error> {
                        Ok(self._is_set_low())
                    }
                }

                impl<MODE> OutputSpeed for $PXi<Output<MODE>> {
                    fn set_speed(&mut self, speed: IOPinSpeed){
                        const OFFSET: u32 = (4 * $i) % 32;

                        cr.cr().modify(|r, w| unsafe {
                            w.bits((r.bits() & !(0b11 << OFFSET)) | ((speed as u32) << OFFSET))
                        });
                    }
                }

                impl OutputSpeed for $PXi<Alternate<PushPull>> {
                    fn set_speed(&mut self, speed: IOPinSpeed){
                        const OFFSET: u32 = (4 * $i) % 32;

                        cr.cr().modify(|r, w| unsafe {
                            w.bits((r.bits() & !(0b11 << OFFSET)) | ((speed as u32) << OFFSET))
                        });
                    }
                }

                impl<MODE> toggleable::Default for $PXi<Output<MODE>> {}

                impl<MODE> InputPin for $PXi<Input<MODE>> {
                    type Error = Infallible;
                    #[inline]
                    fn is_high(&self) -> Result<bool, Self::Error> {
                        self.is_low().map(|b| !b)
                    }

                    #[inline]
                    fn is_low(&self) -> Result<bool, Self::Error> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(self._is_low())
                    }
                }

                impl InputPin for $PXi<Output<OpenDrain>> {
                    type Error = Infallible;
                    #[inline]
                    fn is_high(&self) -> Result<bool, Self::Error> {
                        self.is_low().map(|b| !b)
                    }

                    #[inline]
                    fn is_low(&self) -> Result<bool, Self::Error> {
                        Ok(self._is_low())
                    }
                }

                // Internal helper functions

                // NOTE: The functions in this impl block are "safe", but they
                // are callable when the pin is in modes where they don't make
                // sense.
                impl<MODE> $PXi<MODE> {
                    /**
                      Set the output of the pin regardless of its mode.
                      Primarily used to set the output value of the pin
                      before changing its mode to an output to avoid
                      a short spike of an incorrect value
                    */
                    fn set_state(&mut self, state: State) {
                        match state {
                            State::High => unsafe {
                                (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << $i))
                            }
                            State::Low => unsafe {
                                (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + $i)))
                            }
                        }
                    }

                    fn _is_set_low(&self) -> bool {
                        unsafe { (*$GPIOX::ptr()).odr.read().bits() & (1 << $i) == 0 }
                    }

                    fn _is_low(&self) -> bool {
                        // NOTE(unsafe) atomic read with no side effects
                        unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << $i) == 0 }
                    }
                }

                impl PinMode for $PXi<Input<Floating>> {
                    unsafe fn set_mode() -> Self {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Floating input
                        const CNF: u32 = 0b01;
                        // Input mode
                        const MODE: u32 = 0b00;
                        const BITS: u32 = (CNF << 2) | MODE;

                        // input mode
                        cr
                            .cr()
                            .modify(|r, w| {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { mode: Input::_new() }
                    }
                }

                impl PinMode for $PXi<Input<PullDown>> {
                    unsafe fn set_mode() -> Self {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Pull up/down input
                        const CNF: u32 = 0b10;
                        // Input mode
                        const MODE: u32 = 0b00;
                        const BITS: u32 = (CNF << 2) | MODE;

                        //pull down:
                        // NOTE(unsafe) atomic write to a stateless register
                        (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + $i)));

                        // input mode
                        cr
                            .cr()
                            .modify(|r, w| {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { mode: Input::_new() }
                    }
                }


                impl PinMode for $PXi<Input<PullUp>> {
                    unsafe fn set_mode() -> Self {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Pull up/down input
                        const CNF: u32 = 0b10;
                        // Input mode
                        const MODE: u32 = 0b00;
                        const BITS: u32 = (CNF << 2) | MODE;

                        //pull up:
                        // NOTE(unsafe) atomic write to a stateless register
                        (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << $i));

                        // input mode
                        cr
                            .cr()
                            .modify(|r, w| {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { mode: Input::_new() }
                    }
                }

                impl PinMode for $PXi<Output<OpenDrain>> {
                    unsafe fn set_mode() -> Self {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // General purpose output open-drain
                        const CNF: u32 = 0b01;
                        // Open-Drain Output mode, max speed 50 MHz
                        const MODE: u32 = 0b11;
                        const BITS: u32 = (CNF << 2) | MODE;

                        cr
                            .cr()
                            .modify(|r, w| {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { mode: Output::_new() }
                    }
                }

                impl PinMode for $PXi<Output<PushPull>> {
                    unsafe fn set_mode() -> Self {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // General purpose output push-pull
                        const CNF: u32 = 0b00;
                        // Output mode, max speed 50 MHz
                        const MODE: u32 = 0b11;
                        const BITS: u32 = (CNF << 2) | MODE;


                        cr
                            .cr()
                            .modify(|r, w| {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { mode: Output::_new() }
                    }
                }

                impl PinMode for $PXi<Analog> {
                    unsafe fn set_mode() -> Self {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Analog input
                        const CNF: u32 = 0b00;
                        // Input mode
                        const MODE: u32 = 0b00;
                        const BITS: u32 = (CNF << 2) | MODE;

                        // analog mode
                        cr
                            .cr()
                            .modify(|r, w| {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { mode: Analog{} }
                    }
                }
            )+
        }
    }
}

macro_rules! impl_pxx {
    ($(($port:ident :: $pin:ident)),*) => {
        use embedded_hal::digital::v2::{InputPin, StatefulOutputPin, OutputPin};
        use core::convert::Infallible;

        pub enum Pxx<MODE> {
            $(
                $pin($port::Generic<MODE>)
            ),*
        }

        impl<MODE> OutputPin for Pxx<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.set_high()),*
                }
            }

            fn set_low(&mut self) -> Result<(), Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.set_low()),*
                }
            }
        }

        impl<MODE> StatefulOutputPin for Pxx<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_set_high()),*
                }
            }

            fn is_set_low(&self) -> Result<bool, Self::Error> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_set_low()),*
                }
            }
        }

        impl<MODE> InputPin for Pxx<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_high()),*
                }
            }

            fn is_low(&self) -> Result<bool, Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_low()),*
                }
            }
        }

        impl InputPin for Pxx<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_high()),*
                }
            }

            fn is_low(&self) -> Result<bool, Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_low()),*
                }
            }
        }
    }
}

impl_pxx! {
    (gpioa::PAx),
/*    (gpiob::PBx),
    (gpioc::PCx),
    (gpiod::PDx),
    (gpiof::PFx)
*/}

gpio!(GPIOA, gpioa, paen, PAx, [
    PA0: (pa0, 0, Input<Floating>),
    PA1: (pa1, 1, Input<Floating>),
    PA2: (pa2, 2, Input<Floating>),
    PA3: (pa3, 3, Input<Floating>),
    PA4: (pa4, 4, Input<Floating>),
    PA5: (pa5, 5, Input<Floating>),
    PA6: (pa6, 6, Input<Floating>),
    PA7: (pa7, 7, Input<Floating>),
    PA8: (pa8, 8, Input<Floating>),
    PA9: (pa9, 9, Input<Floating>),
    PA10: (pa10, 10, Input<Floating>),
    PA11: (pa11, 11, Input<Floating>),
    PA12: (pa12, 12, Input<Floating>),
    PA13: (pa13, 13, Debugger),
    PA14: (pa14, 14, Debugger),
    PA15: (pa15, 15, Debugger),
]);

/*gpio!(GPIOB, gpiob, pben, PBx, [
    PB0: (pb0, 0, Input<Floating>),
    PB1: (pb1, 1, Input<Floating>),
    PB2: (pb2, 2, Input<Floating>),
    PB3: (pb3, 3, Debugger),
    PB4: (pb4, 4, Debugger),
    PB5: (pb5, 5, Input<Floating>),
    PB6: (pb6, 6, Input<Floating>),
    PB7: (pb7, 7, Input<Floating>),
    PB8: (pb8, 8, Input<Floating>),
    PB9: (pb9, 9, Input<Floating>),
    PB10: (pb10, 10, Input<Floating>),
    PB11: (pb11, 11, Input<Floating>),
    PB12: (pb12, 12, Input<Floating>),
    PB13: (pb13, 13, Input<Floating>),
    PB14: (pb14, 14, Input<Floating>),
    PB15: (pb15, 15, Input<Floating>),
]);

gpio!(GPIOC, gpioc, pcen, PCx, [
    PC0: (pc0, 0, Input<Floating>),
    PC1: (pc1, 1, Input<Floating>),
    PC2: (pc2, 2, Input<Floating>),
    PC3: (pc3, 3, Input<Floating>),
    PC4: (pc4, 4, Input<Floating>),
    PC5: (pc5, 5, Input<Floating>),
    PC6: (pc6, 6, Input<Floating>),
    PC7: (pc7, 7, Input<Floating>),
    PC8: (pc8, 8, Input<Floating>),
    PC9: (pc9, 9, Input<Floating>),
    PC10: (pc10, 10, Input<Floating>),
    PC11: (pc11, 11, Input<Floating>),
    PC12: (pc12, 12, Input<Floating>),
    PC13: (pc13, 13, Input<Floating>),
    PC14: (pc14, 14, Input<Floating>),
    PC15: (pc15, 15, Input<Floating>),
]);*/
/*
gpio!(GPIOD, gpiod, pden, PDx, [
    PD0: (pd0, 0, Input<Floating>),
    PD1: (pd1, 1, Input<Floating>),
    PD2: (pd2, 2, Input<Floating>),
    PD3: (pd3, 3, Input<Floating>),
    PD4: (pd4, 4, Input<Floating>),
    PD5: (pd5, 5, Input<Floating>),
    PD6: (pd6, 6, Input<Floating>),
    PD7: (pd7, 7, Input<Floating>),
    PD8: (pd8, 8, Input<Floating>),
    PD9: (pd9, 9, Input<Floating>),
    PD10: (pd10, 10, Input<Floating>),
    PD11: (pd11, 11, Input<Floating>),
    PD12: (pd12, 12, Input<Floating>),
    PD13: (pd13, 13, Input<Floating>),
    PD14: (pd14, 14, Input<Floating>),
    PD15: (pd15, 15, Input<Floating>),
]);

gpio!(GPIOF, gpiof, pfen, PFx, [
    PF0: (pf0, 0, Input<Floating>),
    PF1: (pf1, 1, Input<Floating>),
    PF2: (pf2, 2, Input<Floating>),
    PF3: (pf3, 3, Input<Floating>),
    PF4: (pf4, 4, Input<Floating>),
    PF5: (pf5, 5, Input<Floating>),
    PF6: (pf6, 6, Input<Floating>),
    PF7: (pf7, 7, Input<Floating>),
    PF8: (pf8, 8, Input<Floating>),
    PF9: (pf9, 9, Input<Floating>),
    PF10: (pf10, 10, Input<Floating>),
    PF11: (pf11, 11, Input<Floating>),
    PF12: (pf12, 12, Input<Floating>),
    PF13: (pf13, 13, Input<Floating>),
    PF14: (pf14, 14, Input<Floating>),
    PF15: (pf15, 15, Input<Floating>),
]);
*/