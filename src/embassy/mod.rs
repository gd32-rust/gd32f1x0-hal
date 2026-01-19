use time_driver::{Bus, EmbassyTimeDriver, Timer};

use crate::rcu::Clocks;

mod time_driver;

pub fn init(timer: Timer, clocks: &Clocks, apb: &mut Bus) {
    EmbassyTimeDriver::init(timer, clocks, apb)
}
