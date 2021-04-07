#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m::iprintln;
use gd32f1x0_hal as _;
use panic_itm as _;

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();
    let mut itm = p.ITM;

    iprintln!(&mut itm.stim[0], "Hello, world!");

    loop {}
}
