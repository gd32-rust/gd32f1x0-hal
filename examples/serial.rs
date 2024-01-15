//! Serial interface loopback test
//!
//! You have to short the TX and RX pins to make this program work

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;
use cortex_m_rt::entry;
use embedded_hal_02::serial::{Read, Write};
use gd32f1x0_hal::{
    gpio::{OutputMode, PullMode},
    pac,
    prelude::*,
    serial::{Config, Serial},
};
use nb::block;

#[entry]
fn main() -> ! {
    // Get access to the device specific peripherals from the peripheral access crate.
    let p = pac::Peripherals::take().unwrap();

    // Take ownership of the RCU and FMC peripherals and convert them into the corresponding HAL
    // structs.
    let mut rcu = p.RCU.constrain();
    let mut flash = p.FMC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`.
    let clocks = rcu.cfgr.freeze(&mut flash.ws);

    // Prepare the GPIOA peripheral
    let mut gpioa = p.GPIOA.split(&mut rcu.ahb);

    // USART0
    // Configure pa9 and pa10 in alternate function mode for the USART.
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

    // Set up the usart device. Takes ownership of the USART registers and tx/rx pins. The rest of
    // the registers are used to enable and configure the device.
    let mut serial = Serial::usart(
        p.USART0,
        (tx, rx),
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcu.apb2,
    );

    // Loopback test. Write `X` and wait until the write is successful.
    let sent = b'X';
    block!(serial.write(sent)).ok();

    // Read the byte that was just sent. Blocks until the read is complete
    let received = block!(serial.read()).unwrap();

    // Since we have connected tx and rx, the byte we sent should be the one we received
    assert_eq!(received, sent);

    // Trigger a breakpoint to allow us to inspect the values
    asm::bkpt();

    // You can also split the serial struct into a receiving and a transmitting part
    let (mut tx, mut rx) = serial.split();
    let sent = b'Y';
    block!(tx.write(sent)).ok();
    let received = block!(rx.read()).unwrap();
    assert_eq!(received, sent);
    asm::bkpt();

    #[allow(clippy::empty_loop)]
    loop {}
}
