// Copyright 2021 The gd32f1x0-hal authors.
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use crate::rcu::AHB;
use core::convert::Infallible;
use core::marker::PhantomData;
use embedded_hal::digital::v2::{toggleable, InputPin, OutputPin, StatefulOutputPin};

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// Type to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, ahb: &mut AHB) -> Self::Parts;
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
impl AF for AF1 {
    const NUMBER: u32 = 1;
}

/// Alternate function 2
pub struct AF2;
impl AF for AF2 {
    const NUMBER: u32 = 2;
}

/// Alternate function 3
pub struct AF3;
impl AF for AF3 {
    const NUMBER: u32 = 3;
}

/// Alternate function 4
pub struct AF4;
impl AF for AF4 {
    const NUMBER: u32 = 4;
}

/// Alternate function 5
pub struct AF5;
impl AF for AF5 {
    const NUMBER: u32 = 5;
}

/// Alternate function 6
pub struct AF6;
impl AF for AF6 {
    const NUMBER: u32 = 6;
}

/// Alternate function 7
pub struct AF7;
impl AF for AF7 {
    const NUMBER: u32 = 7;
}

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
        unsafe {
            (*self.port).set_high(self.pin_index);
        }
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        unsafe {
            (*self.port).set_low(self.pin_index);
        }
        Ok(())
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

/// Generates core code for a GPIO port, not including alternate function support.
macro_rules! gpio_core {
    ($GPIOX:ident, $gpiox:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty),)+
    ]) => {
        use super::*;
        use crate::pac::{$gpiox, $GPIOX};
        use crate::rcu::Enable;

        /// The pins and overall configuration of the GPIO port.
        pub struct Parts {
            pub config: Config,
            $(
                pub $pxi: $PXi<$MODE>,
            )+
        }

        /// Dummy type representing the configuration of the GPIO port. It must be passed to methods
        /// which reconfigure a pin on the port.
        pub struct Config {
            _config: (),
        }

        impl GpioExt for $GPIOX {
            type Parts = Parts;

            fn split(self, ahb: &mut AHB) -> Parts {
                $GPIOX::enable(ahb);

                Parts {
                    config: Config { _config: () },
                    $(
                        $pxi: $PXi {
                            mode: <$MODE>::new(),
                        },
                    )+
                }
            }
        }

        impl GpioRegExt for $gpiox::RegisterBlock {
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

        /// Sets the input/output, pull-up/down and output mode of a pin
        unsafe fn set_mode(
            _config: &mut Config,
            pin_index: u8,
            ctl: Mode,
            pud: PullMode,
            output_mode: OutputMode,
        ) {
            let offset = 2 * pin_index;
            let reg = &(*$GPIOX::ptr());
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
            let reg = &(*$GPIOX::ptr());
            reg.ospd
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | ((speed as u32) << offset)));
        }

        // Struct for each pin
        $(
            pub struct $PXi<MODE> {
                mode: MODE,
            }

            impl $PXi<Debugger> {
                pub fn activate(self) -> $PXi<Input<Floating>> {
                    $PXi { mode: Input::new() }
                }
            }

            impl<MODE> $PXi<MODE>
            where
                MODE: Active,
            {
                /// Configures the pin to operate as a floating input.
                #[inline]
                pub fn into_floating_input(self, config: &mut Config) -> $PXi<Input<Floating>> {
                    unsafe {
                        set_mode(
                            config,
                            $i,
                            Mode::Input,
                            PullMode::Floating,
                            OutputMode::PushPull,
                        )
                    };
                    $PXi { mode: Input::new() }
                }

                /// Configures the pin to operate as a pulled down input.
                #[inline]
                pub fn into_pull_up_input(self, config: &mut Config) -> $PXi<Input<PullUp>> {
                    unsafe {
                        set_mode(
                            config,
                            $i,
                            Mode::Input,
                            PullMode::PullUp,
                            OutputMode::PushPull,
                        )
                    };
                    $PXi { mode: Input::new() }
                }

                /// Configures the pin to operate as a pulled down input.
                #[inline]
                pub fn into_pull_down_input(self, config: &mut Config) -> $PXi<Input<PullDown>> {
                    unsafe {
                        set_mode(
                            config,
                            $i,
                            Mode::Input,
                            PullMode::PullDown,
                            OutputMode::PushPull,
                        )
                    };
                    $PXi { mode: Input::new() }
                }

                /// Configures the pin to operate as an analog input or output.
                #[inline]
                pub fn into_analog(self, config: &mut Config) -> $PXi<Analog> {
                    unsafe {
                        set_mode(
                            config,
                            $i,
                            Mode::Analog,
                            PullMode::Floating,
                            OutputMode::PushPull,
                        )
                    };
                    $PXi { mode: Analog {} }
                }

                /// Configures the pin to operate as an open drain output.
                #[inline]
                pub fn into_open_drain_output(self, config: &mut Config) -> $PXi<Output<OpenDrain>> {
                    unsafe {
                        set_mode(
                            config,
                            $i,
                            Mode::Output,
                            PullMode::Floating,
                            OutputMode::OpenDrain,
                        )
                    };
                    $PXi {
                        mode: Output::new(),
                    }
                }

                /// Configures the pin to operate as an open drain output.
                #[inline]
                pub fn into_push_pull_output(self, config: &mut Config) -> $PXi<Output<PushPull>> {
                    unsafe {
                        set_mode(
                            config,
                            $i,
                            Mode::Output,
                            PullMode::Floating,
                            OutputMode::PushPull,
                        )
                    };
                    $PXi {
                        mode: Output::new(),
                    }
                }

                /// Erases the pin number and port from the type.
                #[inline]
                pub fn downgrade(self) -> Pin<MODE> {
                    Pin {
                        pin_index: $i,
                        port: $GPIOX::ptr(),
                        _mode: self.mode,
                    }
                }
            }

            impl<MODE> $PXi<Output<MODE>> {
                /// Configures the max slew rate of the pin
                pub fn set_speed(&mut self, config: &mut Config, speed: Speed) {
                    unsafe { set_speed(config, $i, speed) };
                }
            }

            impl<MODE> toggleable::Default for $PXi<Output<MODE>> {}

            impl<MODE> OutputPin for $PXi<Output<MODE>> {
                type Error = Infallible;

                fn set_high(&mut self) -> Result<(), Self::Error> {
                    Ok(unsafe { (*$GPIOX::ptr()).set_high($i) })
                }

                fn set_low(&mut self) -> Result<(), Self::Error> {
                    Ok(unsafe { (*$GPIOX::ptr()).set_low($i) })
                }
            }

            impl<MODE> StatefulOutputPin for $PXi<Output<MODE>> {
                fn is_set_high(&self) -> Result<bool, Self::Error> {
                    self.is_set_low().map(|b| !b)
                }

                fn is_set_low(&self) -> Result<bool, Self::Error> {
                    Ok(unsafe { (*$GPIOX::ptr()).is_set_low($i) })
                }
            }

            impl<MODE> InputPin for $PXi<Input<MODE>> {
                type Error = Infallible;

                fn is_high(&self) -> Result<bool, Self::Error> {
                    self.is_low().map(|b| !b)
                }

                fn is_low(&self) -> Result<bool, Self::Error> {
                    Ok(unsafe { (*$GPIOX::ptr()).is_low($i) })
                }
            }

            impl InputPin for $PXi<Output<OpenDrain>> {
                type Error = Infallible;

                fn is_high(&self) -> Result<bool, Self::Error> {
                    self.is_low().map(|b| !b)
                }

                fn is_low(&self) -> Result<bool, Self::Error> {
                    Ok(unsafe { (*$GPIOX::ptr()).is_low($i) })
                }
            }
        )+
    }
}

/// Generates alternate function code for a GPIO port. This is a separate macro because not all ports
/// support it.
macro_rules! gpio_af {
    ($GPIOX:ident, $gpiox:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty),)+
    ]) => {
        /// Configures the given pin to have the given alternate function mode
        unsafe fn set_alternate_mode(
            config: &mut Config,
            pin_index: u8,
            mode: u32,
            pull_mode: PullMode,
            output_mode: OutputMode,
        ) {
            let offset = (4 * pin_index) % 32;
            let reg = &(*$GPIOX::ptr());
            if pin_index < 8 {
                reg.afsel0
                    .modify(|r, w| w.bits((r.bits() & !(0b1111 << offset)) | (mode << offset)));
            } else {
                reg.afsel1
                    .modify(|r, w| w.bits((r.bits() & !(0b1111 << offset)) | (mode << offset)));
            }
            set_mode(config, pin_index, Mode::Alternate, pull_mode, output_mode);
        }

        // Struct for each pin
        $(
            impl<MODE> $PXi<MODE>
            where
                MODE: Active,
            {
                /// Configures the pin to operate as an alternate function.
                pub fn into_alternate<AFN: AF>(
                    self,
                    config: &mut Config,
                    pull_mode: PullMode,
                    output_mode: OutputMode,
                ) -> $PXi<Alternate<AFN>> {
                    unsafe { set_alternate_mode(config, $i, AFN::NUMBER, pull_mode, output_mode) };
                    $PXi {
                        mode: Alternate::new(),
                    }
                }
            }

            impl<AFN> $PXi<Alternate<AFN>> {
                /// Configures the max slew rate of the pin
                pub fn set_speed(&mut self, config: &mut Config, speed: Speed) {
                    unsafe { set_speed(config, $i, speed) };
                }
            }
        )+
    }
}

/// Generates module for GPIO port with alternate functions.
macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            gpio_core!($GPIOX, $gpiox, [
                $(
                    $PXi: ($pxi, $i, $MODE),
                )+
            ]);
            gpio_af!($GPIOX, $gpiox, [
                $(
                    $PXi: ($pxi, $i, $MODE),
                )+
            ]);
        }
    }
}

/// Generates module for GPIO port without alternate functions.
macro_rules! gpio_noaf {
    ($GPIOX:ident, $gpiox:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            gpio_core!($GPIOX, $gpiox, [
                $(
                    $PXi: ($pxi, $i, $MODE),
                )+
            ]);
        }
    }
}

gpio!(GPIOA, gpioa, [
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
    PA15: (pa15, 15, Input<Floating>),
]);

gpio!(GPIOB, gpiob, [
    PB0: (pb0, 0, Input<Floating>),
    PB1: (pb1, 1, Input<Floating>),
    PB2: (pb2, 2, Input<Floating>),
    PB3: (pb3, 3, Input<Floating>),
    PB4: (pb4, 4, Input<Floating>),
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

gpio!(GPIOC, gpioc, [
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
]);

gpio_noaf!(GPIOD, gpiod, [
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

gpio_noaf!(GPIOF, gpiof, [
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
