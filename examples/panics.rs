//! Prints "Hello, world" on the OpenOCD console

#![no_main]
#![no_std]

use panic_semihosting as _;
//use panic_itm as _;
use cortex_m_semihosting::hprintln;
use gd32f1x0_hal as _;

use cortex_m_rt::{ExceptionFrame, entry, exception};

#[entry]
fn main() -> ! {
    hprintln!("Hello, world!");

    #[allow(clippy::empty_loop)]
    loop {}
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
