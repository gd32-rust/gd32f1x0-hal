#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_semihosting as _;

use cortex_m_rt::entry;
use gd32f1x0_hal::{adc::Adc, pac, prelude::*};

use cortex_m_semihosting::hprintln;

#[entry]
fn main() -> ! {
    // Acquire peripherals
    let p = pac::Peripherals::take().unwrap();
    let mut rcu = p.RCU.constrain();
    let mut flash = p.FMC.constrain();

    let clocks = rcu
        .cfgr
        .use_hxtal(8.mhz())
        .sysclk(56.mhz())
        .pclk1(28.mhz())
        .adcclk(14.mhz())
        .freeze(&mut flash.ws);
    hprintln!("sysclk freq: {}", clocks.sysclk().0);
    hprintln!("adc freq: {}", clocks.adcclk().0);

    // Setup ADC
    let mut adc = Adc::new(p.ADC, &mut rcu.apb2, clocks);

    // Read temperature sensor
    loop {
        let temp = adc.read_temperature();

        hprintln!("temp: {}", temp);
    }
}
