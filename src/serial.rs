//! # Serial Communication (USART)
//!
//! This module contains the functions to utilize the USART (Universal
//! synchronous asynchronous receiver transmitter).

use core::marker::PhantomData;
use core::ops::Deref;
use core::ptr;
use core::sync::atomic::{self, Ordering};

use crate::pac::{rcu::APB2EN, usart0, usart0::ctl1::STB_A, USART0, USART1};
use core::convert::Infallible;
use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};
use embedded_hal::serial::{Read, Write};

use crate::gpio::gpioa::{PA10, PA2, PA3, PA9};
use crate::gpio::gpiob::{PB10, PB11, PB6, PB7};
use crate::gpio::gpioc::{PC10, PC11};
use crate::gpio::gpiod::{PD5, PD6, PD8, PD9};
use crate::gpio::{Alternate, Floating, Input, PushPull, AF1};
use crate::rcc::{sealed::RcuBus, Clocks, Enable, GetBusFreq, Reset, APB1, APB2};
use crate::time::{Bps, U32Ext};

/// Serial error
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
}

pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
}

pub type StopBits = STB_A;

pub struct Config {
    pub baudrate: Bps,
    pub parity: Parity,
    pub stopbits: StopBits,
}

impl Config {
    pub fn baudrate(mut self, baudrate: Bps) -> Self {
        self.baudrate = baudrate;
        self
    }

    pub fn parity_none(mut self) -> Self {
        self.parity = Parity::ParityNone;
        self
    }

    pub fn parity_even(mut self) -> Self {
        self.parity = Parity::ParityEven;
        self
    }

    pub fn parity_odd(mut self) -> Self {
        self.parity = Parity::ParityOdd;
        self
    }

    pub fn stopbits(mut self, stopbits: StopBits) -> Self {
        self.stopbits = stopbits;
        self
    }
}

impl Default for Config {
    fn default() -> Config {
        let baudrate = 115_200_u32.bps();
        Config {
            baudrate,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
        }
    }
}

pub trait TxPin<USART> {}
pub trait RxPin<USART> {}

impl TxPin<USART0> for PA9<Alternate<AF1>> {}
impl TxPin<USART0> for PB6<Alternate<AF1>> {}
impl RxPin<USART0> for PA10<Alternate<AF1>> {}
impl RxPin<USART0> for PB7<Alternate<AF1>> {}

/// Serial abstraction
pub struct Serial<USART, TXPIN, RXPIN> {
    usart: USART,
    pins: (TXPIN, RXPIN),
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

unsafe impl<USART> Send for Rx<USART> {}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

unsafe impl<USART> Send for Tx<USART> {}

impl<TXPIN: TxPin<USART0>, RXPIN: RxPin<USART0>> Serial<USART0, TXPIN, RXPIN> {
    /// Configures the USART and creates a new Serial instance.
    pub fn usart0(
        usart: USART0,
        pins: (TXPIN, RXPIN),
        config: Config,
        clocks: Clocks,
        apb: &mut APB2,
    ) -> Self {
        // Enable clock for USART, and reset it.
        USART0::enable(apb);
        USART0::reset(apb);

        // Configure baud rate.
        let baud_rate_ratio = <USART0 as RcuBus>::Bus::get_frequency(&clocks).0 / config.baudrate.0;
        assert!(baud_rate_ratio >= 16 && baud_rate_ratio <= 0xFFFF);
        usart.baud.write(|w| unsafe { w.bits(baud_rate_ratio) });

        // Configure parity. Note that the parity bit counts towards the word length, so we have to
        // increase it to 9 bits if parity is enabled so as to still get 8 data bits.
        match config.parity {
            Parity::ParityNone => {
                usart.ctl0.modify(|_, w| w.pcen().disabled().wl().bit8());
            }
            Parity::ParityEven => {
                usart
                    .ctl0
                    .modify(|_, w| w.pcen().enabled().wl().bit9().pm().even());
            }
            Parity::ParityOdd => {
                usart
                    .ctl0
                    .modify(|_, w| w.pcen().enabled().wl().bit9().pm().odd());
            }
        }

        // Configure stop bits.
        usart.ctl1.modify(|_, w| w.stb().variant(config.stopbits));

        // Enable transmitter, receiver and the USART as a whole.
        usart
            .ctl0
            .modify(|_, w| w.ten().enabled().ren().enabled().uen().enabled());

        Self { usart, pins }
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>, TXPIN: TxPin<USART>, RXPIN> Write<u8>
    for Serial<USART, TXPIN, RXPIN>
{
    type Error = Infallible;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        let status = self.usart.stat.read();
        if status.tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        let status = self.usart.stat.read();
        if status.tbe().bit_is_set() {
            self.usart
                .tdata
                .write(|w| unsafe { w.tdata().bits(byte.into()) });
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>, TXPIN, RXPIN: RxPin<USART>> Read<u8>
    for Serial<USART, TXPIN, RXPIN>
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        let status = self.usart.stat.read();

        if status.perr().bit_is_set() {
            self.usart.intc.write(|w| w.pec().clear());
            Err(nb::Error::Other(Error::Parity))
        } else if status.ferr().bit_is_set() {
            self.usart.intc.write(|w| w.fec().clear());
            Err(nb::Error::Other(Error::Framing))
        } else if status.nerr().bit_is_set() {
            self.usart.intc.write(|w| w.nec().clear());
            Err(nb::Error::Other(Error::Noise))
        } else if status.orerr().bit_is_set() {
            self.usart.intc.write(|w| w.orec().clear());
            Err(nb::Error::Other(Error::Overrun))
        } else if status.rbne().bit_is_set() {
            Ok(self.usart.rdata.read().rdata().bits() as u8)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}
