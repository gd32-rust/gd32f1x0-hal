//! Testing PWM output for pre-defined pin combination: all pins for default mapping

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;
use cortex_m_rt::entry;
use embedded_hal::pwm::SetDutyCycle;
use gd32f1x0_hal::{
    gpio::{gpioa::PA11, OutputMode, PullMode},
    pac,
    prelude::*,
    pwm::Channel,
    time::U32Ext,
    timer::Timer,
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcu = p.RCU.constrain();
    let mut flash = p.FMC.constrain();

    let clocks = rcu.cfgr.freeze(&mut flash.ws);

    let mut gpioa = p.GPIOA.split(&mut rcu.ahb);

    // TIMER0
    let c0 = gpioa
        .pa8
        .into_alternate(&mut gpioa.config, PullMode::Floating, OutputMode::PushPull);
    let c1 = gpioa
        .pa9
        .into_alternate(&mut gpioa.config, PullMode::Floating, OutputMode::PushPull);
    let c2 = gpioa
        .pa10
        .into_alternate(&mut gpioa.config, PullMode::Floating, OutputMode::PushPull);
    // If you don't want to use all channels, just leave some out
    let pins = (Some(c0), Some(c1), Some(c2), None::<PA11<_>>);

    let mut pwm = Timer::timer0(p.TIMER0, &clocks, &mut rcu.apb2).pwm(pins, 1.khz());

    // Enable clock on each of the channels
    pwm.enable(Channel::C0);
    pwm.enable(Channel::C1);
    pwm.enable(Channel::C2);

    //// Operations affecting all defined channels on the Timer

    // Adjust period to 0.5 seconds
    pwm.set_period(500.ms());

    asm::bkpt();

    // Return to the original frequency
    pwm.set_period(1.khz());

    asm::bkpt();

    let max = pwm.max_duty_cycle();

    //// Operations affecting single channels can be accessed through
    //// the Pwm object or via dereferencing to the pin.

    // Use the Pwm object to set C2 to full strength
    pwm.set_duty_cycle(Channel::C2, max);

    asm::bkpt();

    // Use the Pwm object to set C2 to be dim
    pwm.set_duty_cycle(Channel::C2, max / 4);

    asm::bkpt();

    // Use the Pwm object to set C2 to be zero
    pwm.set_duty_cycle(Channel::C2, 0);

    asm::bkpt();

    // Extract the PwmChannel for C2
    let mut pwm_channel = pwm.split().2.unwrap();

    // Use the PwmChannel object to set C2 to be full strength
    pwm_channel.set_duty_cycle(max).unwrap();

    asm::bkpt();

    // Use the PwmChannel object to set C2 to be dim
    pwm_channel.set_duty_cycle(max / 4).unwrap();

    asm::bkpt();

    // Use the PwmChannel object to set C2 to be zero
    pwm_channel.set_duty_cycle(0).unwrap();

    asm::bkpt();

    #[allow(clippy::empty_loop)]
    loop {}
}
