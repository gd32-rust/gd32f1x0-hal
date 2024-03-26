//! Serial interface write formatted strings test
//!
//! You need to connect the Tx pin to the Rx pin of a serial-usb converter
//! so you can see the message in a serial console (e.g. Arduino console).

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use core::fmt::Write;
use cortex_m_rt::entry;
use gd32f1x0_hal::{
    gpio::{OutputMode, PullMode},
    pac,
    prelude::*,
    serial::{Config, Serial},
};

#[entry]
fn main() -> ! {
    // Get access to the device specific peripherals from the peripheral access crate.
    let p = pac::Peripherals::take().unwrap();

    // Take ownership of the RCU and FMC peripherals and convert them into the corresponding HAL
    // structs.
    let mut rcu = p.rcu.constrain();
    let mut flash = p.fmc.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`.
    let clocks = rcu.cfgr.freeze(&mut flash.ws);

    // Prepare the GPIOA peripheral
    let mut gpioa = p.gpioa.split(&mut rcu.ahb);

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
    let serial = Serial::usart(
        p.usart0,
        (tx, rx),
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcu.apb2,
    );

    // Split the serial struct into a receiving and a transmitting part
    let (mut tx, _rx) = serial.split();

    let number = 103;
    writeln!(tx, "Hello formatted string {}", number).unwrap();

    #[allow(clippy::empty_loop)]
    loop {}
}
