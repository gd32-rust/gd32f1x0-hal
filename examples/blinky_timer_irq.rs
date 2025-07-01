//! blinky timer using interrupts on TIM2
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Please note according to RM0008:
//! "Due to the fact that the switch only sinks a limited amount of current (3 mA), the use of
//! GPIOs PC13 to PC15 in output mode is restricted: the speed has to be limited to 2MHz with
//! a maximum load of 30pF and these IOs must not be used as a current source (e.g. to drive a LED)"

#![no_main]
#![no_std]

use panic_halt as _;

use core::cell::RefCell;
use cortex_m::{asm::wfi, interrupt::Mutex};
use cortex_m_rt::entry;
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use gd32f1x0_hal::{
    gpio::{Output, PushPull, gpioc},
    pac::{Interrupt, Peripherals, Timer1, interrupt},
    prelude::*,
    timer::{CountDownTimer, Event, Timer},
};

// A type definition for the GPIO pin to be used for our LED
type LedPin = gpioc::PC13<Output<PushPull>>;

// Make LED pin globally available
static G_LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));

// Make timer interrupt registers globally available
static G_TIM: Mutex<RefCell<Option<CountDownTimer<Timer1>>>> = Mutex::new(RefCell::new(None));

// Define an interupt handler, i.e. function to call when interrupt occurs.
// This specific interrupt will "trip" when the timer TIMER1 times out
#[interrupt]
fn TIMER1() {
    cortex_m::interrupt::free(|cs| {
        let _ = G_LED.borrow(cs).borrow_mut().as_mut().unwrap().toggle();
        G_TIM
            .borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .clear_interrupt_flag(Event::Update);
    });
}

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.fmc.constrain();
    let mut rcu = dp.rcu.constrain();
    let clocks = rcu
        .cfgr
        .sysclk(8.mhz())
        .pclk1(8.mhz())
        .freeze(&mut flash.ws);

    // Configure PC13 pin to blink LED
    let mut gpioc = dp.gpioc.split(&mut rcu.ahb);
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.config);
    let _ = led.set_high(); // Turn off

    // Move the pin into our global storage
    cortex_m::interrupt::free(|cs| *G_LED.borrow(cs).borrow_mut() = Some(led));

    // Set up a timer expiring after 1s
    let mut timer = Timer::timer1(dp.timer1, &clocks, &mut rcu.apb1).start_count_down(1.hz());

    // Generate an interrupt when the timer expires
    timer.listen(Event::Update);

    // Move the timer into our global storage
    cortex_m::interrupt::free(|cs| *G_TIM.borrow(cs).borrow_mut() = Some(timer));

    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIMER1);
    }

    loop {
        wfi();
    }
}
