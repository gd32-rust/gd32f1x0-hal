//! # Serial Communication (USART)
//!
//! This module contains the functions to utilize the USART (Universal
//! synchronous asynchronous receiver transmitter).

use core::fmt;
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
        Config {
            baudrate: 115_200_u32.bps(),
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
        }
    }
}

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rbne,
    /// New data can be sent
    Tbe,
    /// Idle line state detected
    Idle,
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
    usart: *const usart0::RegisterBlock,
    _instance: PhantomData<USART>,
}

unsafe impl<USART> Send for Rx<USART> {}

/// Serial transmitter
pub struct Tx<USART> {
    usart: *const usart0::RegisterBlock,
    _instance: PhantomData<USART>,
}

unsafe impl<USART> Send for Tx<USART> {}

impl<
        USART: RcuBus + Enable + Reset + Deref<Target = usart0::RegisterBlock>,
        TXPIN: TxPin<USART>,
        RXPIN: RxPin<USART>,
    > Serial<USART, TXPIN, RXPIN>
where
    USART::Bus: GetBusFreq,
{
    /// Configures the USART and creates a new Serial instance.
    pub fn usart(
        usart: USART,
        pins: (TXPIN, RXPIN),
        config: Config,
        clocks: Clocks,
        bus: &mut USART::Bus,
    ) -> Self {
        usart.enable_configure(config, clocks, bus);

        // Enable transmitter, receiver and the USART as a whole.
        usart
            .ctl0
            .modify(|_, w| w.ten().enabled().ren().enabled().uen().enabled());

        Self { usart, pins }
    }

    /// Separates the serial struct into separate channel objects for sending (Tx) and receiving (Rx).
    pub fn split(self) -> (Tx<USART>, Rx<USART>) {
        (
            Tx {
                usart: &*self.usart,
                _instance: PhantomData,
            },
            Rx {
                usart: &*self.usart,
                _instance: PhantomData,
            },
        )
    }
}

impl<
        USART: RcuBus + Enable + Reset + Deref<Target = usart0::RegisterBlock>,
        TXPIN: TxPin<USART>,
    > Serial<USART, TXPIN, ()>
where
    USART::Bus: GetBusFreq,
{
    /// Configures the USART and creates a new TX-only Serial instance.
    pub fn usart_tx(
        usart: USART,
        txpin: TXPIN,
        config: Config,
        clocks: Clocks,
        bus: &mut USART::Bus,
    ) -> Self {
        usart.enable_configure(config, clocks, bus);

        // Enable transmitter and the USART as a whole.
        usart.ctl0.modify(|_, w| w.ten().enabled().uen().enabled());

        Self {
            usart,
            pins: (txpin, ()),
        }
    }

    /// Erase the pin.
    pub fn downgrade_tx(self) -> Tx<USART> {
        Tx {
            usart: &*self.usart,
            _instance: PhantomData,
        }
    }
}

impl<
        USART: RcuBus + Enable + Reset + Deref<Target = usart0::RegisterBlock>,
        RXPIN: RxPin<USART>,
    > Serial<USART, (), RXPIN>
where
    USART::Bus: GetBusFreq,
{
    /// Configures the USART and creates a new RX-only Serial instance.
    pub fn usart_rx(
        usart: USART,
        rxpin: RXPIN,
        config: Config,
        clocks: Clocks,
        bus: &mut USART::Bus,
    ) -> Self {
        usart.enable_configure(config, clocks, bus);

        // Enable receiver and the USART as a whole.
        usart.ctl0.modify(|_, w| w.ren().enabled().uen().enabled());

        Self {
            usart,
            pins: ((), rxpin),
        }
    }

    /// Erase the pin.
    pub fn downgrade_rx(self) -> Rx<USART> {
        Rx {
            usart: &*self.usart,
            _instance: PhantomData,
        }
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>, TXPIN, RXPIN> Serial<USART, TXPIN, RXPIN> {
    pub fn release(self) -> (USART, (TXPIN, RXPIN)) {
        (self.usart, self.pins)
    }

    /// Enable an interrupt event.
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Rbne => self.usart.ctl0.modify(|_, w| w.rbneie().enabled()),
            Event::Tbe => self.usart.ctl0.modify(|_, w| w.tbeie().enabled()),
            Event::Idle => self.usart.ctl0.modify(|_, w| w.idleie().enabled()),
        }
    }

    /// Disable an interrupt event.
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Rbne => self.usart.ctl0.modify(|_, w| w.rbneie().disabled()),
            Event::Tbe => self.usart.ctl0.modify(|_, w| w.tbeie().disabled()),
            Event::Idle => self.usart.ctl0.modify(|_, w| w.idleie().disabled()),
        }
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>> Rx<USART> {
    /// Enable the RBNE interrupt.
    pub fn listen(&mut self) {
        unsafe { &*self.usart }
            .ctl0
            .modify(|_, w| w.rbneie().enabled());
    }

    /// Disable the RBNE interrupt.
    pub fn unlisten(&mut self) {
        unsafe { &*self.usart }
            .ctl0
            .modify(|_, w| w.rbneie().disabled());
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>> Tx<USART> {
    /// Enable the TBE interrupt.
    pub fn listen(&mut self) {
        unsafe { &*self.usart }
            .ctl0
            .modify(|_, w| w.tbeie().enabled());
    }

    /// Disable the TBE interrupt.
    pub fn unlisten(&mut self) {
        unsafe { &*self.usart }
            .ctl0
            .modify(|_, w| w.tbeie().disabled());
    }
}

// Implement writing traits if the USART has a TX pin assigned.
impl<USART: Deref<Target = usart0::RegisterBlock>, TXPIN: TxPin<USART>, RXPIN> Write<u8>
    for Serial<USART, TXPIN, RXPIN>
{
    type Error = Infallible;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.usart.flush()
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        self.usart.write(byte)
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>> Write<u8> for Tx<USART> {
    type Error = Infallible;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        unsafe { &*self.usart }.flush()
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        unsafe { &*self.usart }.write(byte)
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>, TXPIN: TxPin<USART>, RXPIN> fmt::Write
    for Serial<USART, TXPIN, RXPIN>
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        s.as_bytes()
            .iter()
            .try_for_each(|c| nb::block!(self.write(*c)))
            .map_err(|_| core::fmt::Error)
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>> fmt::Write for Tx<USART> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        s.as_bytes()
            .iter()
            .try_for_each(|c| nb::block!(self.write(*c)))
            .map_err(|_| core::fmt::Error)
    }
}

// Implement reading trait if the USART has an RX pin assigned.
impl<USART: Deref<Target = usart0::RegisterBlock>, TXPIN, RXPIN: RxPin<USART>> Read<u8>
    for Serial<USART, TXPIN, RXPIN>
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        self.usart.read()
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>> Read<u8> for Rx<USART> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        unsafe { &*self.usart }.read()
    }
}

trait UsartConfigExt {
    type Bus;

    fn enable_configure(&self, config: Config, clocks: Clocks, bus: &mut Self::Bus);
}

impl<USART: RcuBus + Enable + Reset + Deref<Target = usart0::RegisterBlock>> UsartConfigExt
    for USART
where
    USART::Bus: GetBusFreq,
{
    type Bus = USART::Bus;

    /// Enable, reset and configure the USART.
    fn enable_configure(&self, config: Config, clocks: Clocks, bus: &mut Self::Bus) {
        // Enable clock for USART, and reset it.
        USART::enable(bus);
        USART::reset(bus);

        // Configure baud rate.
        let baud_rate_ratio = <USART as RcuBus>::Bus::get_frequency(&clocks).0 / config.baudrate.0;
        assert!(baud_rate_ratio >= 16 && baud_rate_ratio <= 0xFFFF);
        self.baud.write(|w| unsafe { w.bits(baud_rate_ratio) });

        // Configure parity. Note that the parity bit counts towards the word length, so we have to
        // increase it to 9 bits if parity is enabled so as to still get 8 data bits.
        match config.parity {
            Parity::ParityNone => {
                self.ctl0.modify(|_, w| w.pcen().disabled().wl().bit8());
            }
            Parity::ParityEven => {
                self.ctl0
                    .modify(|_, w| w.pcen().enabled().wl().bit9().pm().even());
            }
            Parity::ParityOdd => {
                self.ctl0
                    .modify(|_, w| w.pcen().enabled().wl().bit9().pm().odd());
            }
        }

        // Configure stop bits.
        self.ctl1.modify(|_, w| w.stb().variant(config.stopbits));
    }
}

trait UsartReadWrite {
    fn read(&mut self) -> nb::Result<u8, Error>;
    fn flush(&mut self) -> nb::Result<(), Infallible>;
    fn write(&mut self, byte: u8) -> nb::Result<(), Infallible>;
}

impl<USART: Deref<Target = usart0::RegisterBlock>> UsartReadWrite for USART {
    fn read(&mut self) -> nb::Result<u8, Error> {
        let status = self.stat.read();

        if status.perr().bit_is_set() {
            self.intc.write(|w| w.pec().clear());
            Err(nb::Error::Other(Error::Parity))
        } else if status.ferr().bit_is_set() {
            self.intc.write(|w| w.fec().clear());
            Err(nb::Error::Other(Error::Framing))
        } else if status.nerr().bit_is_set() {
            self.intc.write(|w| w.nec().clear());
            Err(nb::Error::Other(Error::Noise))
        } else if status.orerr().bit_is_set() {
            self.intc.write(|w| w.orec().clear());
            Err(nb::Error::Other(Error::Overrun))
        } else if status.rbne().bit_is_set() {
            Ok(self.rdata.read().rdata().bits() as u8)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn flush(&mut self) -> nb::Result<(), Infallible> {
        let status = self.stat.read();
        if status.tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Infallible> {
        let status = self.stat.read();
        if status.tbe().bit_is_set() {
            self.tdata.write(|w| unsafe { w.tdata().bits(byte.into()) });
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}
