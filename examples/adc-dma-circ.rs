//! ADC interface circular DMA RX transfer test

#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::{asm, singleton};

use cortex_m_rt::entry;
use gd32f1x0_hal::{adc::Adc, dma::Half, pac, prelude::*};

#[entry]
fn main() -> ! {
    // Acquire peripherals
    let p = pac::Peripherals::take().unwrap();
    let mut rcu = p.RCU.constrain();
    let mut flash = p.FMC.constrain();

    // Configure ADC clocks
    // Default value is the slowest possible ADC clock: PCLK2 / 8. Meanwhile ADC
    // clock is configurable. So its frequency may be tweaked to meet certain
    // practical needs. User specified value is be approximated using supported
    // prescaler values 2/4/6/8.
    let clocks = rcu.cfgr.adcclk(2.mhz()).freeze(&mut flash.ws);

    let dma_ch0 = p.DMA.split(&mut rcu.ahb).0;

    // Setup ADC
    let adc = Adc::new(p.ADC, &mut rcu.apb2, clocks);

    // Setup GPIOA
    let mut gpioa = p.GPIOA.split(&mut rcu.ahb);

    // Configure pa0 as an analog input
    let adc_ch0 = gpioa.pa0.into_analog(&mut gpioa.config);

    let adc_dma = adc.with_dma(adc_ch0, dma_ch0);
    let buf = singleton!(: [[u16; 8]; 2] = [[0; 8]; 2]).unwrap();

    let mut circ_buffer = adc_dma.circ_read(buf);

    while circ_buffer.readable_half().unwrap() != Half::First {}

    let _first_half = circ_buffer.peek(|half, _| *half).unwrap();

    while circ_buffer.readable_half().unwrap() != Half::Second {}

    let _second_half = circ_buffer.peek(|half, _| *half).unwrap();

    let (_buf, adc_dma) = circ_buffer.stop();
    let (_adc1, _adc_ch0, _dma_ch1) = adc_dma.split();
    asm::bkpt();

    loop {}
}
