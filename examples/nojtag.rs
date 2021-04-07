//! Disables the JTAG ports to give access to pb3, pb4 and PA15

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use gd32f1x0_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcu = p.RCU.constrain();
    let mut gpioa = p.GPIOA.split(&mut rcu.ahb);

    // If you really want to use a JTAG pin for something else, you must first call `.activate()`.
    let mut pa13 = gpioa
        .pa13
        .activate()
        .into_push_pull_output(&mut gpioa.config);
    let mut pa14 = gpioa
        .pa14
        .activate()
        .into_push_pull_output(&mut gpioa.config);

    loop {
        pa13.toggle().unwrap();
        pa14.toggle().unwrap();
        cortex_m::asm::delay(8_000_000);
    }
}
