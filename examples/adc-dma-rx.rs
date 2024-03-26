//! ADC interface DMA RX transfer test

#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::{asm, singleton};
use cortex_m_rt::entry;
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

    let dma_ch0 = p.dma.split(&mut rcu.ahb).0;

    // Setup ADC
    let adc = Adc::new(p.adc, &mut rcu.apb2, clocks);

    // Setup GPIOA
    let mut gpioa = p.gpioa.split(&mut rcu.ahb);

    // Configure pa0 as an analog input
    let adc_ch0 = gpioa.pa0.into_analog(&mut gpioa.config);

    let adc_dma = adc.with_dma(adc_ch0, dma_ch0);
    let buf = singleton!(: [u16; 8] = [0; 8]).unwrap();

    // The read method consumes the buf and self, starts the adc and dma transfer and returns a
    // RxDma struct. The wait method consumes the RxDma struct, waits for the whole transfer to be
    // completed and then returns the updated buf and underlying adc_dma struct. For non blocking,
    // one can call the is_done method of RxDma and only call wait after that method returns true.
    let (_buf, adc_dma) = adc_dma.read(buf).wait();
    asm::bkpt();

    // Consumes the AdcDma struct, restores adc configuration to previous state and returns the
    // Adc struct in normal mode.
    let (_adc, _adc_ch0, _dma_ch1) = adc_dma.split();
    asm::bkpt();

    #[allow(clippy::empty_loop)]
    loop {}
}
