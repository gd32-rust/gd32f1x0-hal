//! Turns an LED on
//!
//! This assumes that an active high LED is connected to pc9.

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use embedded_hal::digital::OutputPin;
use gd32f1x0_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcu = p.RCU.constrain();
    let mut gpioc = p.GPIOC.split(&mut rcu.ahb);

    gpioc
        .pc9
        .into_push_pull_output(&mut gpioc.config)
        .set_high()
        .unwrap();

    #[allow(clippy::empty_loop)]
    loop {}
}
