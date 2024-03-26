//! Blinks several LEDs stored in an array

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use core::hint::spin_loop;
use cortex_m_rt::entry;
use embedded_hal::digital::OutputPin;
use gd32f1x0_hal::{pac, prelude::*, timer::Timer};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut rcu = dp.rcu.constrain();
    let mut flash = dp.fmc.constrain();

    let clocks = rcu.cfgr.freeze(&mut flash.ws);

    // Acquire the GPIO peripherals
    let mut gpioa = dp.gpioa.split(&mut rcu.ahb);
    let mut gpioc = dp.gpioc.split(&mut rcu.ahb);

    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(cp.SYST, &clocks).start_count_down(1.hz());

    // Create an array of LEDS to blink
    let mut leds = [
        gpioc
            .pc13
            .into_push_pull_output(&mut gpioc.config)
            .downgrade(),
        gpioa
            .pa1
            .into_push_pull_output(&mut gpioa.config)
            .downgrade(),
    ];

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        while !timer.has_elapsed() {
            spin_loop();
        }
        for led in leds.iter_mut() {
            led.set_high().unwrap();
        }
        while !timer.has_elapsed() {
            spin_loop();
        }
        for led in leds.iter_mut() {
            led.set_low().unwrap();
        }
    }
}
