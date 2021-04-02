//! CRC

use crate::pac::CRC;
use crate::rcu::{Enable, AHB};

/// Extension trait to constrain the CRC peripheral
pub trait CrcExt {
    /// Constrains the CRC peripheral to play nicely with the other abstractions
    fn new(self, ahb: &mut AHB) -> Crc;
}

impl CrcExt for CRC {
    fn new(self, ahb: &mut AHB) -> Crc {
        CRC::enable(ahb);
        Crc { crc: self }
    }
}

/// Constrained CRC peripheral
pub struct Crc {
    crc: CRC,
}

impl Crc {
    pub fn read(&self) -> u32 {
        self.crc.data.read().bits()
    }

    pub fn write(&mut self, val: u32) {
        self.crc.data.write(|w| w.data().bits(val))
    }

    pub fn reset(&self) {
        self.crc.ctl.write(|w| w.rst().reset());
        // calling CRC::dr::write() just after CRC::ctl::reset() will not work as expected, and
        // inserting single nop() seems to solve the problem.
        cortex_m::asm::nop();
    }
}
