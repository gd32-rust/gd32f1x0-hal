//! Inter-Integrated Circuit (I2C) bus

// This document describes a correct i2c implementation and is what
// parts of this code is based on
// https://www.st.com/content/ccc/resource/technical/document/application_note/5d/ae/a3/6f/08/69/4e/9b/CD00209826.pdf/files/CD00209826.pdf/jcr:content/translations/en.CD00209826.pdf

use crate::gpio::gpioa::{PA0, PA1, PA10, PA9};
use crate::gpio::gpiob::{PB10, PB11, PB6, PB7, PB8, PB9};
use crate::gpio::gpioc::{PC0, PC1, PC7, PC8};
use crate::gpio::gpiof::{PF6, PF7};
use crate::gpio::{Alternate, AF0, AF1, AF4};
#[cfg(any(feature = "gd32f170x8", feature = "gd32f190x8"))]
use crate::pac::I2C2;
use crate::pac::{DWT, I2C0, I2C1};
use crate::rcu::{Clocks, Enable, GetBusFreq, Reset, APB1};
use crate::time::Hertz;
use core::ops::Deref;
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use nb::Error::{Other, WouldBlock};
use nb::{Error as NbError, Result as NbResult};

/// I2C error
#[derive(Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// No ack received
    Acknowledge,
    /// Overrun/underrun
    Overrun,
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
}

#[derive(Debug, Eq, PartialEq)]
pub enum DutyCycle {
    Ratio2to1,
    Ratio16to9,
}

#[derive(Debug, PartialEq)]
pub enum Mode {
    Standard {
        frequency: Hertz,
    },
    Fast {
        frequency: Hertz,
        duty_cycle: DutyCycle,
    },
}

impl Mode {
    pub fn standard<F: Into<Hertz>>(frequency: F) -> Self {
        Mode::Standard {
            frequency: frequency.into(),
        }
    }

    pub fn fast<F: Into<Hertz>>(frequency: F, duty_cycle: DutyCycle) -> Self {
        Mode::Fast {
            frequency: frequency.into(),
            duty_cycle,
        }
    }

    pub fn get_frequency(&self) -> Hertz {
        match *self {
            Mode::Standard { frequency } => frequency,
            Mode::Fast { frequency, .. } => frequency,
        }
    }
}

/// Helper trait to ensure that the correct I2C pins are used for the corresponding interface
pub trait Pins<I2C> {}

//SCL, SDA
impl Pins<I2C0> for (PA9<Alternate<AF4>>, PA10<Alternate<AF4>>) {}
impl Pins<I2C0> for (PB6<Alternate<AF1>>, PB7<Alternate<AF1>>) {}
impl Pins<I2C0> for (PB8<Alternate<AF1>>, PB9<Alternate<AF1>>) {}
#[cfg(any(feature = "gd32f170x4", feature = "gd32f190x4"))]
impl Pins<I2C0> for (PB10<Alternate<AF1>>, PB11<Alternate<AF1>>) {}
#[cfg(any(
    feature = "gd32f130x4",
    feature = "gd32f130x6",
    feature = "gd32f170x4",
    feature = "gd32f190x4",
))]
impl Pins<I2C0> for (PF6<Alternate<AF0>>, PF7<Alternate<AF0>>) {}

#[cfg(any(
    feature = "gd32f130x8",
    feature = "gd32f150x8",
    feature = "gd32f170x8",
    feature = "gd32f190x8",
))]
impl Pins<I2C1> for (PA0<Alternate<AF4>>, PA1<Alternate<AF4>>) {}
#[cfg(any(
    feature = "gd32f130x8",
    feature = "gd32f150x8",
    feature = "gd32f170x8",
    feature = "gd32f190x8",
))]
impl Pins<I2C1> for (PB10<Alternate<AF1>>, PB11<Alternate<AF1>>) {}
#[cfg(any(feature = "gd32f130x8", feature = "gd32f170x8", feature = "gd32f190x8"))]
impl Pins<I2C1> for (PF6<Alternate<AF0>>, PF7<Alternate<AF0>>) {}

#[cfg(any(feature = "gd32f170x8", feature = "gd32f190x8"))]
impl Pins<I2C2> for (PB6<Alternate<AF4>>, PB7<Alternate<AF4>>) {}
#[cfg(any(feature = "gd32f170x8", feature = "gd32f190x8"))]
impl Pins<I2C2> for (PC0<Alternate<AF1>>, PC1<Alternate<AF1>>) {}
#[cfg(any(feature = "gd32f170x8", feature = "gd32f190x8"))]
impl Pins<I2C2> for (PC7<Alternate<AF1>>, PC8<Alternate<AF1>>) {}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
    mode: Mode,
    pclk1: u32,
}

/// embedded-hal compatible blocking I2C implementation
///
/// **NOTE**: Before using blocking I2C, you need to enable the DWT cycle counter using the
/// [DWT::enable_cycle_counter] method.
pub struct BlockingI2c<I2C, PINS> {
    nb: I2c<I2C, PINS>,
    start_timeout: u32,
    start_retries: u8,
    addr_timeout: u32,
    data_timeout: u32,
}

impl<PINS> I2c<I2C0, PINS> {
    /// Creates a generic I2C0 object on the given pins.
    pub fn i2c0(i2c: I2C0, pins: PINS, mode: Mode, clocks: Clocks, apb: &mut APB1) -> Self
    where
        PINS: Pins<I2C0>,
    {
        I2c::<I2C0, _>::_i2c(i2c, pins, mode, clocks, apb)
    }
}

impl<PINS> BlockingI2c<I2C0, PINS> {
    /// Creates a blocking I2C0 object on the given pins using the embedded-hal `BlockingI2c` trait.
    pub fn i2c0(
        i2c: I2C0,
        pins: PINS,
        mode: Mode,
        clocks: Clocks,
        apb: &mut APB1,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> Self
    where
        PINS: Pins<I2C0>,
    {
        BlockingI2c::<I2C0, _>::_i2c(
            i2c,
            pins,
            mode,
            clocks,
            apb,
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
        )
    }
}

impl<PINS> I2c<I2C1, PINS> {
    /// Creates a generic I2C1 object on the given pins using the embedded-hal `BlockingI2c` trait.
    pub fn i2c1(i2c: I2C1, pins: PINS, mode: Mode, clocks: Clocks, apb: &mut APB1) -> Self
    where
        PINS: Pins<I2C1>,
    {
        I2c::<I2C1, _>::_i2c(i2c, pins, mode, clocks, apb)
    }
}

impl<PINS> BlockingI2c<I2C1, PINS> {
    /// Creates a blocking I2C1 object on the given pins.
    pub fn i2c1(
        i2c: I2C1,
        pins: PINS,
        mode: Mode,
        clocks: Clocks,
        apb: &mut APB1,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> Self
    where
        PINS: Pins<I2C1>,
    {
        BlockingI2c::<I2C1, _>::_i2c(
            i2c,
            pins,
            mode,
            clocks,
            apb,
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
        )
    }
}

/// Generates a blocking I2C instance from a universal I2C object
fn blocking_i2c<I2C, PINS>(
    i2c: I2c<I2C, PINS>,
    clocks: Clocks,
    start_timeout_us: u32,
    start_retries: u8,
    addr_timeout_us: u32,
    data_timeout_us: u32,
) -> BlockingI2c<I2C, PINS> {
    let sysclk_mhz = clocks.sysclk().0 / 1_000_000;
    BlockingI2c {
        nb: i2c,
        start_timeout: start_timeout_us * sysclk_mhz,
        start_retries,
        addr_timeout: addr_timeout_us * sysclk_mhz,
        data_timeout: data_timeout_us * sysclk_mhz,
    }
}

macro_rules! wait_for_flag {
    ($i2c:expr, $flag:ident) => {{
        let stat0 = $i2c.stat0.read();

        if stat0.berr().bit_is_set() {
            $i2c.stat0.write(|w| w.berr().clear_bit());
            Err(Other(Error::Bus))
        } else if stat0.lostarb().bit_is_set() {
            $i2c.stat0.write(|w| w.lostarb().clear_bit());
            Err(Other(Error::Arbitration))
        } else if stat0.aerr().bit_is_set() {
            $i2c.stat0.write(|w| w.aerr().clear_bit());
            Err(Other(Error::Acknowledge))
        } else if stat0.ouerr().bit_is_set() {
            $i2c.stat0.write(|w| w.ouerr().clear_bit());
            Err(Other(Error::Overrun))
        } else if stat0.$flag().bit_is_set() {
            Ok(())
        } else {
            Err(WouldBlock)
        }
    }};
}

macro_rules! busy_wait {
    ($nb_expr:expr, $exit_cond:expr) => {{
        loop {
            let res = $nb_expr;
            if res != Err(WouldBlock) {
                break res;
            }
            if $exit_cond {
                break res;
            }
        }
    }};
}

macro_rules! busy_wait_cycles {
    ($nb_expr:expr, $cycles:expr) => {{
        let started = DWT::get_cycle_count();
        let cycles = $cycles;
        busy_wait!(
            $nb_expr,
            DWT::get_cycle_count().wrapping_sub(started) >= cycles
        )
    }};
}

pub type I2cRegisterBlock = crate::pac::i2c0::RegisterBlock;

impl<I2C, PINS> I2c<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock> + Enable + Reset,
    I2C::Bus: GetBusFreq,
{
    /// Configures the I2C peripheral to work in master mode
    fn _i2c(i2c: I2C, pins: PINS, mode: Mode, clocks: Clocks, apb: &mut I2C::Bus) -> Self {
        I2C::enable(apb);
        I2C::reset(apb);

        let pclk1 = I2C::Bus::get_frequency(&clocks).0;

        assert!(mode.get_frequency().0 <= 400_000);

        let mut i2c = I2c {
            i2c,
            pins,
            mode,
            pclk1,
        };
        i2c.init();
        i2c
    }
}

impl<I2C, PINS> I2c<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    /// Initializes I2C. Configures the `I2C_RT`, `I2C_CTLn`, and `I2C_CKCFG` registers
    /// according to the system frequency and I2C mode.
    fn init(&mut self) {
        let freq = self.mode.get_frequency();
        let pclk1_mhz = (self.pclk1 / 1000000) as u16;

        self.i2c
            .ctl1
            .write(|w| unsafe { w.i2cclk().bits(pclk1_mhz as u8) });
        self.i2c.ctl0.write(|w| w.i2cen().disabled());

        match self.mode {
            Mode::Standard { .. } => {
                self.i2c
                    .rt
                    .write(|w| unsafe { w.risetime().bits((pclk1_mhz + 1) as u8) });
                self.i2c
                    .ckcfg
                    .write(|w| w.clkc().bits(((self.pclk1 / (freq.0 * 2)) as u16).max(4)));
            }
            Mode::Fast { ref duty_cycle, .. } => {
                self.i2c
                    .rt
                    .write(|w| unsafe { w.risetime().bits((pclk1_mhz * 300 / 1000 + 1) as u8) });

                self.i2c.ckcfg.write(|w| {
                    let (freq, duty) = match duty_cycle {
                        &DutyCycle::Ratio2to1 => {
                            (((self.pclk1 / (freq.0 * 3)) as u16).max(1), false)
                        }
                        &DutyCycle::Ratio16to9 => {
                            (((self.pclk1 / (freq.0 * 25)) as u16).max(1), true)
                        }
                    };

                    w.clkc().bits(freq).dtcy().bit(duty).fast().fast()
                });
            }
        };

        self.i2c.ctl0.modify(|_, w| w.i2cen().set_bit());
    }

    /// Perform an I2C software reset
    fn reset(&mut self) {
        self.i2c
            .ctl0
            .write(|w| w.i2cen().enabled().sreset().reset());
        self.i2c.ctl0.reset();
        self.init();
    }

    /// Generate START condition
    fn send_start(&mut self) {
        self.i2c.ctl0.modify(|_, w| w.start().set_bit());
    }

    /// Check if START condition is generated. If the condition is not generated, this
    /// method returns `WouldBlock` so the program can act accordingly
    /// (busy wait, async, ...)
    fn wait_after_sent_start(&mut self) -> NbResult<(), Error> {
        wait_for_flag!(self.i2c, sbsend)
    }

    /// Check if STOP condition is generated. If the condition is not generated, this
    /// method returns `WouldBlock` so the program can act accordingly
    /// (busy wait, async, ...)
    fn wait_for_stop(&mut self) -> NbResult<(), Error> {
        if self.i2c.ctl0.read().stop().is_no_stop() {
            Ok(())
        } else {
            Err(WouldBlock)
        }
    }

    /// Sends the (7-Bit) address on the I2C bus. The 8th bit on the bus is set
    /// depending on wether it is a read or write transfer.
    fn send_addr(&self, addr: u8, read: bool) {
        self.i2c
            .data
            .write(|w| w.trb().bits(addr << 1 | (if read { 1 } else { 0 })));
    }

    /// Generate STOP condition
    fn send_stop(&self) {
        self.i2c.ctl0.modify(|_, w| w.stop().set_bit());
    }

    /// Releases the I2C peripheral and associated pins
    pub fn free(self) -> (I2C, PINS) {
        (self.i2c, self.pins)
    }
}

impl<I2C, PINS> BlockingI2c<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock> + Enable + Reset,
    I2C::Bus: GetBusFreq,
{
    fn _i2c(
        i2c: I2C,
        pins: PINS,
        mode: Mode,
        clocks: Clocks,
        apb: &mut I2C::Bus,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> Self {
        blocking_i2c(
            I2c::<I2C, _>::_i2c(i2c, pins, mode, clocks, apb),
            clocks,
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
        )
    }
}

impl<I2C, PINS> BlockingI2c<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    fn send_start_and_wait(&mut self) -> NbResult<(), Error> {
        // According to http://www.st.com/content/ccc/resource/technical/document/errata_sheet/f5/50/c9/46/56/db/4a/f6/CD00197763.pdf/files/CD00197763.pdf/jcr:content/translations/en.CD00197763.pdf
        // 2.14.4 Wrong behavior of I2C peripheral in master mode after a misplaced STOP
        let mut retries_left = self.start_retries;
        let mut last_ret: NbResult<(), Error> = Err(WouldBlock);
        while retries_left > 0 {
            self.nb.send_start();
            last_ret = busy_wait_cycles!(self.nb.wait_after_sent_start(), self.start_timeout);
            if let Err(_) = last_ret {
                self.nb.reset();
            } else {
                break;
            }
            retries_left -= 1;
        }
        last_ret
    }

    fn send_addr_and_wait(&mut self, addr: u8, read: bool) -> NbResult<(), Error> {
        self.nb.i2c.stat0.read();
        self.nb.send_addr(addr, read);
        let ret = busy_wait_cycles!(wait_for_flag!(self.nb.i2c, addsend), self.addr_timeout);
        if ret == Err(Other(Error::Acknowledge)) {
            self.nb.send_stop();
        }
        ret
    }

    fn write_bytes_and_wait(&mut self, bytes: &[u8]) -> NbResult<(), Error> {
        self.nb.i2c.stat0.read();
        self.nb.i2c.stat1.read();

        self.nb.i2c.data.write(|w| w.trb().bits(bytes[0]));

        for byte in &bytes[1..] {
            busy_wait_cycles!(wait_for_flag!(self.nb.i2c, tbe), self.data_timeout)?;
            self.nb.i2c.data.write(|w| w.trb().bits(*byte));
        }
        busy_wait_cycles!(wait_for_flag!(self.nb.i2c, btc), self.data_timeout)?;

        Ok(())
    }

    fn write_without_stop(&mut self, addr: u8, bytes: &[u8]) -> NbResult<(), Error> {
        self.send_start_and_wait()?;
        self.send_addr_and_wait(addr, false)?;

        let ret = self.write_bytes_and_wait(bytes);
        if ret == Err(Other(Error::Acknowledge)) {
            self.nb.send_stop();
        }
        ret
    }
}

impl<I2C, PINS> Write for BlockingI2c<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    type Error = NbError<Error>;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_without_stop(addr, bytes)?;
        self.nb.send_stop();
        busy_wait_cycles!(self.nb.wait_for_stop(), self.data_timeout)?;

        Ok(())
    }
}

impl<I2C, PINS> Read for BlockingI2c<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    type Error = NbError<Error>;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.send_start_and_wait()?;
        self.send_addr_and_wait(addr, true)?;

        match buffer.len() {
            1 => {
                self.nb.i2c.ctl0.modify(|_, w| w.acken().clear_bit());
                self.nb.i2c.stat0.read();
                self.nb.i2c.stat1.read();
                self.nb.send_stop();

                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rbne), self.data_timeout)?;
                buffer[0] = self.nb.i2c.data.read().trb().bits();

                busy_wait_cycles!(self.nb.wait_for_stop(), self.data_timeout)?;
                self.nb.i2c.ctl0.modify(|_, w| w.acken().set_bit());
            }
            2 => {
                self.nb
                    .i2c
                    .ctl0
                    .modify(|_, w| w.poap().set_bit().acken().set_bit());
                self.nb.i2c.stat0.read();
                self.nb.i2c.stat1.read();
                self.nb.i2c.ctl0.modify(|_, w| w.acken().clear_bit());

                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, btc), self.data_timeout)?;
                self.nb.send_stop();
                buffer[0] = self.nb.i2c.data.read().trb().bits();
                buffer[1] = self.nb.i2c.data.read().trb().bits();

                busy_wait_cycles!(self.nb.wait_for_stop(), self.data_timeout)?;
                self.nb
                    .i2c
                    .ctl0
                    .modify(|_, w| w.poap().clear_bit().acken().clear_bit());
                self.nb.i2c.ctl0.modify(|_, w| w.acken().set_bit());
            }
            buffer_len => {
                self.nb.i2c.ctl0.modify(|_, w| w.acken().set_bit());
                self.nb.i2c.stat0.read();
                self.nb.i2c.stat1.read();

                let (first_bytes, last_two_bytes) = buffer.split_at_mut(buffer_len - 3);
                for byte in first_bytes {
                    busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rbne), self.data_timeout)?;
                    *byte = self.nb.i2c.data.read().trb().bits();
                }

                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, btc), self.data_timeout)?;
                self.nb.i2c.ctl0.modify(|_, w| w.acken().clear_bit());
                last_two_bytes[0] = self.nb.i2c.data.read().trb().bits();
                self.nb.send_stop();
                last_two_bytes[1] = self.nb.i2c.data.read().trb().bits();
                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rbne), self.data_timeout)?;
                last_two_bytes[2] = self.nb.i2c.data.read().trb().bits();

                busy_wait_cycles!(self.nb.wait_for_stop(), self.data_timeout)?;
                self.nb.i2c.ctl0.modify(|_, w| w.acken().set_bit());
            }
        }

        Ok(())
    }
}

impl<I2C, PINS> WriteRead for BlockingI2c<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    type Error = NbError<Error>;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        if !bytes.is_empty() {
            self.write_without_stop(addr, bytes)?;
        }

        if !buffer.is_empty() {
            self.read(addr, buffer)?;
        } else if !bytes.is_empty() {
            self.nb.send_stop();
            busy_wait_cycles!(self.nb.wait_for_stop(), self.data_timeout)?;
        }

        Ok(())
    }
}
