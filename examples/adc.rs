#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_semihosting as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use gd32f1x0_hal::{adc::Adc, pac, prelude::*};

#[entry]
fn main() -> ! {
    // Acquire peripherals
    let p = pac::Peripherals::take().unwrap();
    let mut rcu = p.rcu.constrain();
    let mut flash = p.fmc.constrain();

    // Configure ADC clocks
    // Default value is the slowest possible ADC clock: PCLK2 / 8. Meanwhile ADC
    // clock is configurable. So its frequency may be tweaked to meet certain
    // practical needs. User specified value is be approximated using supported
    // prescaler values 2/4/6/8.
    let clocks = rcu.cfgr.adcclk(2.mhz()).freeze(&mut flash.ws);
    hprintln!("adc freq: {}", clocks.adcclk().0);

    // Setup ADC
    let mut adc = Adc::new(p.adc, &mut rcu.apb2, clocks);

    // Setup GPIOB
    let mut gpiob = p.gpiob.split(&mut rcu.ahb);

    // Configure pb0 as an analog input
    let ch0 = gpiob.pb0.into_analog(&mut gpiob.config);

    loop {
        let data: u16 = adc.read_channel(&ch0);
        hprintln!("adc1: {}", data);
    }
}
