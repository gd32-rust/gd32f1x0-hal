//! Serial interface DMA TX transfer test

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;

use cortex_m_rt::entry;
use gd32f1x0_hal::{
    gpio::{OutputMode, PullMode},
    pac,
    prelude::*,
    serial::{Config, Serial},
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut flash = p.fmc.constrain();
    let mut rcu = p.rcu.constrain();

    let clocks = rcu.cfgr.freeze(&mut flash.ws);

    let channels = p.dma.split(&mut rcu.ahb);

    let mut gpioa = p.gpioa.split(&mut rcu.ahb);
    // let mut gpiob = p.gpiob.split(&mut rcu.ahb);

    // USART0
    let tx = gpioa
        .pa9
        .into_alternate(&mut gpioa.config, PullMode::Floating, OutputMode::PushPull);
    let rx = gpioa
        .pa10
        .into_alternate(&mut gpioa.config, PullMode::Floating, OutputMode::PushPull);

    // USART1
    // let tx = gpioa
    //     .pa2
    //     .into_alternate(&mut gpioa.config, PullMode::Floating, OutputMode::PushPull);
    // let rx = gpioa
    //     .pa3
    //     .into_alternate(&mut gpioa.config, PullMode::Floating, OutputMode::PushPull);

    let serial = Serial::usart(
        p.usart0,
        (tx, rx),
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcu.apb2,
    );

    let tx = serial.split().0.with_dma(channels.1);

    let (_, tx) = tx.write(b"The quick brown fox").wait();

    asm::bkpt();

    let (_, tx) = tx.write(b" jumps").wait();

    asm::bkpt();

    tx.write(b" over the lazy dog.").wait();

    asm::bkpt();

    #[allow(clippy::empty_loop)]
    loop {}
}
