//! Testing PWM with complementary outputs.

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;
use cortex_m_rt::entry;
use embedded_hal::pwm::SetDutyCycle;
use gd32f1x0_hal::{
    gpio::{OutputMode, PullMode},
    pac,
    prelude::*,
    pwm::{Channel, IdleState, Polarity},
    time::U32Ext,
    timer::Timer,
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcu = p.rcu.constrain();
    let mut flash = p.fmc.constrain();

    let clocks = rcu.cfgr.freeze(&mut flash.ws);

    let mut gpioa = p.gpioa.split(&mut rcu.ahb);
    let mut gpiob = p.gpiob.split(&mut rcu.ahb);

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
    let cn0 =
        gpiob
            .pb13
            .into_alternate(&mut gpiob.config, PullMode::Floating, OutputMode::PushPull);
    let cn1 =
        gpiob
            .pb14
            .into_alternate(&mut gpiob.config, PullMode::Floating, OutputMode::PushPull);
    let cn2 =
        gpiob
            .pb15
            .into_alternate(&mut gpiob.config, PullMode::Floating, OutputMode::PushPull);
    let pins = (Some((c0, cn0)), Some((c1, cn1)), Some((c2, cn2)));

    let mut pwm = Timer::timer0(p.timer0, &clocks, &mut rcu.apb2).pwm(pins, 1.khz());

    // Configure polarity for C2.
    pwm.set_polarity(Channel::C2, Polarity::NotInverted);
    pwm.set_complementary_polarity(Channel::C2, Polarity::Inverted);

    // Configure idle state for C2.
    pwm.set_idle_state(Channel::C2, IdleState::Low);
    pwm.set_complementary_idle_state(Channel::C2, IdleState::High);

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
