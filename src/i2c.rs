//! Inter-Integrated Circuit (I2C) bus

// This document describes a correct I2C implementation and is what parts of this code is based on:
// https://www.st.com/content/ccc/resource/technical/document/application_note/5d/ae/a3/6f/08/69/4e/9b/CD00209826.pdf/files/CD00209826.pdf/jcr:content/translations/en.CD00209826.pdf

use crate::gpio::gpioa::*;
use crate::gpio::gpiob::*;
#[cfg(any(feature = "gd32f170x8", feature = "gd32f190x8"))]
use crate::gpio::gpioc::{PC0, PC1, PC7, PC8};
#[cfg(any(
    feature = "gd32f130x4",
    feature = "gd32f130x6",
    feature = "gd32f130x8",
    feature = "gd32f170x4",
    feature = "gd32f170x8",
    feature = "gd32f190x4",
    feature = "gd32f190x8",
))]
use crate::gpio::gpiof::{PF6, PF7};
#[cfg(any(
    feature = "gd32f130x4",
    feature = "gd32f130x6",
    feature = "gd32f130x8",
    feature = "gd32f170x4",
    feature = "gd32f170x8",
    feature = "gd32f190x4",
    feature = "gd32f190x8",
))]
use crate::gpio::AF0;
use crate::gpio::{Alternate, AF1, AF4};
#[cfg(any(feature = "gd32f170x8", feature = "gd32f190x8"))]
use crate::pac::I2C2;
use crate::pac::{DWT, I2C0, I2C1};
#[cfg(any(feature = "gd32f170x8", feature = "gd32f190x8"))]
use crate::rcu::ADDAPB1;
use crate::rcu::{Clocks, Enable, GetBusFreq, Reset, APB1};
use crate::time::Hertz;
use core::ops::Deref;
use embedded_hal::i2c::{ErrorKind, ErrorType, NoAcknowledgeSource, Operation, SevenBitAddress};
use embedded_hal_02::blocking::i2c::{Read, Write, WriteRead};

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
    /// Timed out waiting for something.
    Timeout,
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::Bus => ErrorKind::Bus,
            Self::Arbitration => ErrorKind::ArbitrationLoss,
            Self::Acknowledge => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown),
            Self::Overrun => ErrorKind::Overrun,
            Self::Timeout => ErrorKind::Other,
        }
    }
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

/// Marker trait for possible SCL pins for an I2C module.
pub trait SclPin<I2C> {}

/// Marker trait for possible SDA pins for an I2C module.
pub trait SdaPin<I2C> {}

// Pins for I2C0
impl SclPin<I2C0> for PA9<Alternate<AF4>> {}
impl SdaPin<I2C0> for PA10<Alternate<AF4>> {}
impl SclPin<I2C0> for PB6<Alternate<AF1>> {}
impl SdaPin<I2C0> for PB7<Alternate<AF1>> {}
impl SclPin<I2C0> for PB8<Alternate<AF1>> {}
impl SdaPin<I2C0> for PB9<Alternate<AF1>> {}
#[cfg(any(feature = "gd32f170x4", feature = "gd32f190x4"))]
impl SclPin<I2C0> for PB10<Alternate<AF1>> {}
#[cfg(any(feature = "gd32f170x4", feature = "gd32f190x4"))]
impl SdaPin<I2C0> for PB11<Alternate<AF1>> {}
#[cfg(any(
    feature = "gd32f130x4",
    feature = "gd32f130x6",
    feature = "gd32f170x4",
    feature = "gd32f190x4",
))]
impl SclPin<I2C0> for PF6<Alternate<AF0>> {}
#[cfg(any(
    feature = "gd32f130x4",
    feature = "gd32f130x6",
    feature = "gd32f170x4",
    feature = "gd32f190x4",
))]
impl SdaPin<I2C0> for PF7<Alternate<AF0>> {}

// Pins for I2C1
#[cfg(any(
    feature = "gd32f130x8",
    feature = "gd32f150x8",
    feature = "gd32f170x8",
    feature = "gd32f190x8",
))]
impl SclPin<I2C1> for PA0<Alternate<AF4>> {}
#[cfg(any(
    feature = "gd32f130x8",
    feature = "gd32f150x8",
    feature = "gd32f170x8",
    feature = "gd32f190x8",
))]
impl SdaPin<I2C1> for PA1<Alternate<AF4>> {}
#[cfg(any(
    feature = "gd32f130x8",
    feature = "gd32f150x8",
    feature = "gd32f170x8",
    feature = "gd32f190x8",
))]
impl SclPin<I2C1> for PB10<Alternate<AF1>> {}
#[cfg(any(
    feature = "gd32f130x8",
    feature = "gd32f150x8",
    feature = "gd32f170x8",
    feature = "gd32f190x8",
))]
impl SdaPin<I2C1> for PB11<Alternate<AF1>> {}
#[cfg(any(feature = "gd32f130x8", feature = "gd32f170x8", feature = "gd32f190x8"))]
impl SclPin<I2C1> for PF6<Alternate<AF0>> {}
#[cfg(any(feature = "gd32f130x8", feature = "gd32f170x8", feature = "gd32f190x8"))]
impl SdaPin<I2C1> for PF7<Alternate<AF0>> {}

// Pins for I2C2
#[cfg(any(feature = "gd32f170x8", feature = "gd32f190x8"))]
impl SclPin<I2C2> for PB6<Alternate<AF4>> {}
#[cfg(any(feature = "gd32f170x8", feature = "gd32f190x8"))]
impl SdaPin<I2C2> for PB7<Alternate<AF4>> {}
#[cfg(any(feature = "gd32f170x8", feature = "gd32f190x8"))]
impl SclPin<I2C2> for PC0<Alternate<AF1>> {}
#[cfg(any(feature = "gd32f170x8", feature = "gd32f190x8"))]
impl SdaPin<I2C2> for PC1<Alternate<AF1>> {}
#[cfg(any(feature = "gd32f170x8", feature = "gd32f190x8"))]
impl SclPin<I2C2> for PC7<Alternate<AF1>> {}
#[cfg(any(feature = "gd32f170x8", feature = "gd32f190x8"))]
impl SdaPin<I2C2> for PC8<Alternate<AF1>> {}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, SCLPIN, SDAPIN> {
    i2c: I2C,
    scl_pin: SCLPIN,
    sda_pin: SDAPIN,
    mode: Mode,
    pclk1: u32,
}

/// embedded-hal compatible blocking I2C implementation
///
/// **NOTE**: Before using blocking I2C, you need to enable the DWT cycle counter using the
/// [DWT::enable_cycle_counter] method.
pub struct BlockingI2c<I2C, SCLPIN, SDAPIN> {
    nb: I2c<I2C, SCLPIN, SDAPIN>,
    start_timeout: u32,
    start_retries: u8,
    addr_timeout: u32,
    data_timeout: u32,
}

macro_rules! i2c_impl {
    ($I2Cn:ty, $i2cn:ident, $APBn:ty) => {
        impl<SCLPIN, SDAPIN> I2c<$I2Cn, SCLPIN, SDAPIN> {
            /// Creates a generic I2Cn object on the given pins.
            pub fn $i2cn(
                i2c: $I2Cn,
                scl_pin: SCLPIN,
                sda_pin: SDAPIN,
                mode: Mode,
                clocks: Clocks,
                apb: &mut $APBn,
            ) -> Self
            where
                SCLPIN: SclPin<$I2Cn>,
                SDAPIN: SdaPin<$I2Cn>,
            {
                I2c::<$I2Cn, _, _>::create_internal(i2c, scl_pin, sda_pin, mode, clocks, apb)
            }
        }

        impl<SCLPIN, SDAPIN> BlockingI2c<$I2Cn, SCLPIN, SDAPIN> {
            /// Creates a blocking I2Cn object on the given pins using the embedded-hal `BlockingI2c` trait.
            #[allow(clippy::too_many_arguments)]
            pub fn $i2cn(
                i2c: $I2Cn,
                scl_pin: SCLPIN,
                sda_pin: SDAPIN,
                mode: Mode,
                clocks: Clocks,
                apb: &mut $APBn,
                start_timeout_us: u32,
                start_retries: u8,
                addr_timeout_us: u32,
                data_timeout_us: u32,
            ) -> Self
            where
                SCLPIN: SclPin<$I2Cn>,
                SDAPIN: SdaPin<$I2Cn>,
            {
                BlockingI2c::<$I2Cn, _, _>::create_internal(
                    i2c,
                    scl_pin,
                    sda_pin,
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
    };
}

i2c_impl!(I2C0, i2c0, APB1);
i2c_impl!(I2C1, i2c1, APB1);
#[cfg(any(feature = "gd32f170x8", feature = "gd32f190x8"))]
i2c_impl!(I2C2, i2c2, ADDAPB1);

/// Generates a blocking I2C instance from a universal I2C object.
fn blocking_i2c<I2C, SCLPIN, SDAPIN>(
    i2c: I2c<I2C, SCLPIN, SDAPIN>,
    clocks: Clocks,
    start_timeout_us: u32,
    start_retries: u8,
    addr_timeout_us: u32,
    data_timeout_us: u32,
) -> BlockingI2c<I2C, SCLPIN, SDAPIN> {
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

        if stat0.berr().is_error() {
            $i2c.stat0.write(|w| w.berr().no_error());
            Err(nb::Error::Other(Error::Bus))
        } else if stat0.lostarb().is_lost() {
            $i2c.stat0.write(|w| w.lostarb().no_lost());
            Err(nb::Error::Other(Error::Arbitration))
        } else if stat0.aerr().is_error() {
            $i2c.stat0.write(|w| w.aerr().no_error());
            Err(nb::Error::Other(Error::Acknowledge))
        } else if stat0.ouerr().is_overrun() {
            $i2c.stat0.write(|w| w.ouerr().no_overrun());
            Err(nb::Error::Other(Error::Overrun))
        } else if stat0.$flag().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }};
}

macro_rules! busy_wait {
    ($nb_expr:expr, $exit_cond:expr) => {{
        loop {
            let res = $nb_expr;
            if res != Err(nb::Error::WouldBlock) {
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
        let started = DWT::cycle_count();
        let cycles = $cycles;
        match busy_wait!($nb_expr, DWT::cycle_count().wrapping_sub(started) >= cycles) {
            Ok(v) => Ok(v),
            Err(nb::Error::WouldBlock) => Err(Error::Timeout),
            Err(nb::Error::Other(e)) => Err(e),
        }
    }};
}

pub type I2cRegisterBlock = crate::pac::i2c0::RegisterBlock;

impl<I2C, SCLPIN, SDAPIN> I2c<I2C, SCLPIN, SDAPIN>
where
    I2C: Deref<Target = I2cRegisterBlock> + Enable + Reset,
    I2C::Bus: GetBusFreq,
{
    /// Configures the I2C peripheral to work in master mode
    fn create_internal(
        i2c: I2C,
        scl_pin: SCLPIN,
        sda_pin: SDAPIN,
        mode: Mode,
        clocks: Clocks,
        apb: &mut I2C::Bus,
    ) -> Self {
        I2C::enable(apb);
        I2C::reset(apb);

        let pclk1 = I2C::Bus::get_frequency(&clocks).0;

        assert!(mode.get_frequency().0 <= 400_000);

        let mut i2c = I2c {
            i2c,
            scl_pin,
            sda_pin,
            mode,
            pclk1,
        };
        i2c.init();
        i2c
    }
}

impl<I2C, SCLPIN, SDAPIN> I2c<I2C, SCLPIN, SDAPIN>
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
                        DutyCycle::Ratio2to1 => {
                            (((self.pclk1 / (freq.0 * 3)) as u16).max(1), false)
                        }
                        DutyCycle::Ratio16to9 => {
                            (((self.pclk1 / (freq.0 * 25)) as u16).max(1), true)
                        }
                    };

                    w.clkc().bits(freq).dtcy().bit(duty).fast().fast()
                });
            }
        };

        self.i2c.ctl0.modify(|_, w| w.i2cen().enabled());
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
        self.i2c.ctl0.modify(|_, w| w.start().start());
    }

    /// Check if START condition is generated. If the condition is not generated, this
    /// method returns `WouldBlock` so the program can act accordingly
    /// (busy wait, async, ...)
    fn wait_after_sent_start(&mut self) -> nb::Result<(), Error> {
        wait_for_flag!(self.i2c, sbsend)
    }

    /// Check if STOP condition is generated. If the condition is not generated, this
    /// method returns `WouldBlock` so the program can act accordingly
    /// (busy wait, async, ...)
    fn wait_for_stop(&mut self) -> nb::Result<(), Error> {
        if self.i2c.ctl0.read().stop().is_no_stop() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
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
        self.i2c.ctl0.modify(|_, w| w.stop().stop());
    }

    /// Releases the I2C peripheral and associated pins
    pub fn free(self) -> (I2C, SCLPIN, SDAPIN) {
        (self.i2c, self.scl_pin, self.sda_pin)
    }
}

impl<I2C, SCLPIN, SDAPIN> BlockingI2c<I2C, SCLPIN, SDAPIN>
where
    I2C: Deref<Target = I2cRegisterBlock> + Enable + Reset,
    I2C::Bus: GetBusFreq,
{
    #[allow(clippy::too_many_arguments)]
    fn create_internal(
        i2c: I2C,
        scl_pin: SCLPIN,
        sda_pin: SDAPIN,
        mode: Mode,
        clocks: Clocks,
        apb: &mut I2C::Bus,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> Self {
        blocking_i2c(
            I2c::<I2C, _, _>::create_internal(i2c, scl_pin, sda_pin, mode, clocks, apb),
            clocks,
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
        )
    }
}

impl<I2C, SCLPIN, SDAPIN> BlockingI2c<I2C, SCLPIN, SDAPIN>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    fn send_start_and_wait(&mut self) -> Result<(), Error> {
        // According to http://www.st.com/content/ccc/resource/technical/document/errata_sheet/f5/50/c9/46/56/db/4a/f6/CD00197763.pdf/files/CD00197763.pdf/jcr:content/translations/en.CD00197763.pdf
        // 2.14.4 Wrong behavior of I2C peripheral in master mode after a misplaced STOP
        let mut retries_left = self.start_retries;
        let mut last_ret: Result<(), Error> = Err(Error::Timeout);
        while retries_left > 0 {
            self.nb.send_start();
            last_ret = busy_wait_cycles!(self.nb.wait_after_sent_start(), self.start_timeout);
            if last_ret.is_err() {
                self.nb.reset();
            } else {
                break;
            }
            retries_left -= 1;
        }
        last_ret
    }

    fn send_addr_and_wait(&mut self, addr: u8, read: bool) -> Result<(), Error> {
        self.nb.i2c.stat0.read();
        self.nb.send_addr(addr, read);
        let ret = busy_wait_cycles!(wait_for_flag!(self.nb.i2c, addsend), self.addr_timeout);
        if ret == Err(Error::Acknowledge) {
            self.nb.send_stop();
        }
        ret
    }

    fn write_bytes_and_wait(&mut self, bytes: &[u8]) -> Result<(), Error> {
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

    fn write_without_start_or_stop(&mut self, bytes: &[u8]) -> Result<(), Error> {
        let ret = self.write_bytes_and_wait(bytes);
        if ret == Err(Error::Acknowledge) {
            self.nb.send_stop();
        }
        ret
    }

    fn write_without_stop(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        self.send_start_and_wait()?;
        self.send_addr_and_wait(addr, false)?;
        self.write_without_start_or_stop(bytes)
    }

    fn read_without_start(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        match buffer.len() {
            1 => {
                self.nb.i2c.ctl0.modify(|_, w| w.acken().nak());
                self.nb.i2c.stat0.read();
                self.nb.i2c.stat1.read();
                self.nb.send_stop();

                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rbne), self.data_timeout)?;
                buffer[0] = self.nb.i2c.data.read().trb().bits();

                busy_wait_cycles!(self.nb.wait_for_stop(), self.data_timeout)?;
                self.nb.i2c.ctl0.modify(|_, w| w.acken().ack());
            }
            2 => {
                self.nb
                    .i2c
                    .ctl0
                    .modify(|_, w| w.poap().next().acken().ack());
                self.nb.i2c.stat0.read();
                self.nb.i2c.stat1.read();
                self.nb.i2c.ctl0.modify(|_, w| w.acken().nak());

                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, btc), self.data_timeout)?;
                self.nb.send_stop();
                buffer[0] = self.nb.i2c.data.read().trb().bits();
                buffer[1] = self.nb.i2c.data.read().trb().bits();

                busy_wait_cycles!(self.nb.wait_for_stop(), self.data_timeout)?;
                self.nb
                    .i2c
                    .ctl0
                    .modify(|_, w| w.poap().current().acken().nak());
                self.nb.i2c.ctl0.modify(|_, w| w.acken().ack());
            }
            buffer_len => {
                self.nb.i2c.ctl0.modify(|_, w| w.acken().ack());
                self.nb.i2c.stat0.read();
                self.nb.i2c.stat1.read();

                let (first_bytes, last_three_bytes) = buffer.split_at_mut(buffer_len - 3);
                for byte in first_bytes {
                    busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rbne), self.data_timeout)?;
                    *byte = self.nb.i2c.data.read().trb().bits();
                }

                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, btc), self.data_timeout)?;
                self.nb.i2c.ctl0.modify(|_, w| w.acken().nak());
                last_three_bytes[0] = self.nb.i2c.data.read().trb().bits();
                self.nb.send_stop();
                last_three_bytes[1] = self.nb.i2c.data.read().trb().bits();
                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rbne), self.data_timeout)?;
                last_three_bytes[2] = self.nb.i2c.data.read().trb().bits();

                busy_wait_cycles!(self.nb.wait_for_stop(), self.data_timeout)?;
                self.nb.i2c.ctl0.modify(|_, w| w.acken().ack());
            }
        }

        Ok(())
    }

    fn read_without_start_or_stop(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.nb.i2c.ctl0.modify(|_, w| w.acken().ack());
        self.nb.i2c.stat0.read();
        self.nb.i2c.stat1.read();

        for byte in buffer {
            busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rbne), self.data_timeout)?;
            *byte = self.nb.i2c.data.read().trb().bits();
        }

        Ok(())
    }
}

impl<I2C, SCLPIN, SDAPIN> ErrorType for BlockingI2c<I2C, SCLPIN, SDAPIN>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    type Error = Error;
}

impl<I2C, SCLPIN, SDAPIN> embedded_hal::i2c::I2c<SevenBitAddress>
    for BlockingI2c<I2C, SCLPIN, SDAPIN>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation],
    ) -> Result<(), Self::Error> {
        let operations_count = operations.len();
        // `Some(true)` if the last operation was a read, `Some(false)` if it was a write.
        let mut last_operation_read = None;
        for (i, operation) in operations.into_iter().enumerate() {
            match operation {
                Operation::Read(buffer) => {
                    if last_operation_read != Some(true) {
                        self.send_start_and_wait()?;
                        self.send_addr_and_wait(address, true)?;
                    }
                    if i == operations_count - 1 {
                        // This also skips acknowledgement of the last byte, as required by the spec.
                        self.read_without_start(buffer)?;
                    } else {
                        self.read_without_start_or_stop(buffer)?;
                    }
                    last_operation_read = Some(true);
                }
                Operation::Write(buffer) => {
                    if last_operation_read != Some(false) {
                        self.send_start_and_wait()?;
                        self.send_addr_and_wait(address, false)?;
                    }
                    self.write_without_start_or_stop(buffer)?;
                    if i == operations_count - 1 {
                        self.nb.send_stop();
                        busy_wait_cycles!(self.nb.wait_for_stop(), self.data_timeout)?;
                    }
                    last_operation_read = Some(false);
                }
            }
        }
        Ok(())
    }
}

impl<I2C, SCLPIN, SDAPIN> Write for BlockingI2c<I2C, SCLPIN, SDAPIN>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_without_stop(addr, bytes)?;
        self.nb.send_stop();
        busy_wait_cycles!(self.nb.wait_for_stop(), self.data_timeout)?;

        Ok(())
    }
}

impl<I2C, SCLPIN, SDAPIN> Read for BlockingI2c<I2C, SCLPIN, SDAPIN>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.send_start_and_wait()?;
        self.send_addr_and_wait(addr, true)?;
        self.read_without_start(buffer)
    }
}

impl<I2C, SCLPIN, SDAPIN> WriteRead for BlockingI2c<I2C, SCLPIN, SDAPIN>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    type Error = Error;

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
