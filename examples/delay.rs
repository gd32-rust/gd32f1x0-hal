//! "Blinky" using delays instead of a timer

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use embedded_hal::digital::OutputPin;
use embedded_hal_02::blocking::delay::DelayMs;
use gd32f1x0_hal::{delay::Delay, pac, prelude::*};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut rcu = dp.RCU.constrain();
    let mut flash = dp.FMC.constrain();

    let clocks = rcu.cfgr.freeze(&mut flash.ws);

    let mut gpioc = dp.GPIOC.split(&mut rcu.ahb);

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.config);

    let mut delay = Delay::new(cp.SYST, clocks);

    loop {
        led.set_high().unwrap();
        delay.delay_ms(1_000_u16);
        led.set_low().unwrap();
        delay.delay_ms(1_000_u16);
    }
}
