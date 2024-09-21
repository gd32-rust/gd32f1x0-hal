#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use embassy_executor::{self, Spawner};
use embassy_time::Timer;
use embedded_hal::digital::OutputPin;
use gd32f1x0_hal::{embassy, pac, prelude::*, time::MilliSeconds, watchdog::FreeWatchdog};

#[embassy_executor::task]
async fn blink_task(mut led: impl OutputPin + 'static) {
    loop {
        Timer::after_millis(1_000).await;
        led.set_high().unwrap();
        Timer::after_millis(1_000).await;
        led.set_low().unwrap();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = pac::Peripherals::take().unwrap();
    let mut rcu = p.rcu.constrain();
    let mut flash = p.fmc.constrain();
    let clocks = rcu.cfgr.freeze(&mut flash.ws);

    embassy::init(p.timer1, &clocks, &mut rcu.apb1);

    let mut gpioc = p.gpioc.split(&mut rcu.ahb);
    let led = gpioc
        .pc13
        .into_push_pull_output(&mut gpioc.config)
        .downgrade();

    // This task will run in parallel with the loop below time-sharing MCU resources
    spawner.must_spawn(blink_task(led));

    let mut watchdog = FreeWatchdog::new(p.fwdgt);
    watchdog.start(MilliSeconds(100));
    loop {
        Timer::after_micros(500).await;
        watchdog.feed();
    }
}
