//! Uses the timer interrupt to blink a led with different frequencies.
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Note: Without additional hardware, PC13 should not be used to drive an LED, see page 5.1.2 of
//! the reference manual for an explanation. This is not an issue on the blue pill.

#![no_std]
#![no_main]

// you can put a breakpoint on `rust_begin_unwind` to catch panics
use panic_halt as _;

use rtic::app;

use embedded_hal::digital::OutputPin;
use embedded_hal_02::timer::CountDown;
use gd32f1x0_hal::{
    gpio::{gpioc::PC13, Output, PushPull},
    pac,
    prelude::*,
    timer::{CountDownTimer, Event, Timer},
};

pub struct Led {
    led: PC13<Output<PushPull>>,
    led_state: bool,
}

#[app(device = gd32f1x0_hal::pac, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        timer_handler: CountDownTimer<pac::TIMER0>,
        led: Led,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Take ownership over the raw flash and rcu devices and convert them into the corresponding
        // HAL structs
        let mut flash = cx.device.FMC.constrain();
        let mut rcu = cx.device.RCU.constrain();

        // Freeze the configuration of all the clocks in the system and store the frozen frequencies
        // in `clocks`
        let clocks = rcu.cfgr.freeze(&mut flash.ws);

        // Acquire the GPIOC peripheral
        let mut gpioc = cx.device.GPIOC.split(&mut rcu.ahb);

        // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the
        // function in order to configure the port. For pins 0-7, crl should be passed instead
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.config);
        led.set_high().unwrap();
        // Configure TIMER0 to trigger an update every second and enables interrupt
        let mut timer =
            Timer::timer0(cx.device.TIMER0, &clocks, &mut rcu.apb2).start_count_down(1.hz());
        timer.listen(Event::Update);

        // Init the static resources to use them later through RTIC
        (
            Shared {
                led: Led {
                    led,
                    led_state: false,
                },
                timer_handler: timer,
            },
            Local {},
            init::Monotonics(),
        )
    }

    // Optional.
    //
    // https://rtic.rs/0.5/book/en/by-example/app.html#idle
    // > When no idle function is declared, the runtime sets the SLEEPONEXIT bit and then
    // > sends the microcontroller to sleep after running init.
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = TIMER0_BRK_UP_TRG_COM, priority = 1, local = [count: u8 = 0], shared = [led, timer_handler])]
    fn tick(cx: tick::Context) {
        // Depending on the application, you could want to delegate some of the work done here to
        // the idle task if you want to minimize the latency of interrupts with same priority (if
        // you have any). That could be done with some kind of machine state, etc.

        let mut led = cx.shared.led;
        let mut timer_handler = cx.shared.timer_handler;
        // Count used to change the timer update frequency
        let count = cx.local.count;

        led.lock(|led| {
            if led.led_state {
                // Uses resources managed by rtic to turn led off.
                led.led.set_high().unwrap();
                led.led_state = false;
            } else {
                led.led.set_low().unwrap();
                led.led_state = true;
            }
        });
        *count += 1;

        timer_handler.lock(|timer_handler| {
            if *count == 4 {
                // Changes timer update frequency
                timer_handler.start(2.hz());
            } else if *count == 12 {
                timer_handler.start(1.hz());
                *count = 0;
            }

            // Clears the update flag
            timer_handler.clear_interrupt_flag(Event::Update);
        });
    }
}
