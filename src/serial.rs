// Copyright 2021 The gd32f1x0-hal authors.
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! # Serial Communication (USART)
//!
//! This module contains the functions to utilize the USART (Universal
//! synchronous asynchronous receiver transmitter).

use crate::dma::{
    self, CircBuffer, CircReadDma, Priority, R, ReadDma, Receive, RxDma, Transfer, TransferPayload,
    Transmit, TxDma, W, Width, WriteDma,
};
use crate::gpio::gpioa::{PA2, PA3, PA9, PA10, PA14, PA15};
use crate::gpio::gpiob::{PB6, PB7};
use crate::gpio::{AF0, AF1, Alternate};
use crate::pac::{self, Usart0, usart0, usart0::ctl1::Stb};
use crate::rcu::{Clocks, Enable, GetBusFreq, Reset, sealed::RcuBus};
use crate::time::{Bps, U32Ext};
use core::convert::Infallible;
use core::fmt;
use core::hint::spin_loop;
use core::marker::PhantomData;
use core::ops::Deref;
use core::sync::atomic::{self, Ordering};
use embedded_dma::{ReadBuffer, WriteBuffer};
use embedded_io::{ErrorKind, ErrorType, Read, ReadReady, Write, WriteReady};

/// Serial error
#[derive(Copy, Clone, Debug, Eq, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    #[error("Framing error")]
    Framing,
    /// Noise error
    #[error("Noise error")]
    Noise,
    /// RX buffer overrun
    #[error("RX buffer overrun")]
    Overrun,
    /// Parity check error
    #[error("Parity check error")]
    Parity,
}

impl From<Infallible> for Error {
    fn from(e: Infallible) -> Self {
        match e {}
    }
}

impl embedded_io::Error for Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::Framing | Self::Noise | Self::Parity => ErrorKind::InvalidData,
            Self::Overrun => ErrorKind::Other,
        }
    }
}

pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
}

pub type StopBits = Stb;

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
            stopbits: StopBits::Stop1,
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

impl TxPin<Usart0> for PA9<Alternate<AF1>> {}
impl RxPin<Usart0> for PA10<Alternate<AF1>> {}
impl TxPin<Usart0> for PB6<Alternate<AF0>> {}
impl RxPin<Usart0> for PB7<Alternate<AF0>> {}

// Two USARTs
#[cfg(any(
    feature = "gd32f130x6",
    feature = "gd32f130x8",
    feature = "gd32f150x6",
    feature = "gd32f150x8",
    feature = "gd32f170x6",
    feature = "gd32f170x8",
    feature = "gd32f190x6",
    feature = "gd32f190x8",
))]
mod pins {
    use super::*;
    use crate::gpio::gpiob::PB0;
    use crate::gpio::{AF4, gpioa::PA8};
    use crate::pac::Usart1;

    impl TxPin<Usart1> for PA2<Alternate<AF1>> {}
    impl RxPin<Usart1> for PA3<Alternate<AF1>> {}
    impl TxPin<Usart1> for PA8<Alternate<AF4>> {}
    impl RxPin<Usart1> for PB0<Alternate<AF4>> {}
    impl TxPin<Usart1> for PA14<Alternate<AF1>> {}
    impl RxPin<Usart1> for PA15<Alternate<AF1>> {}
}

// Only one USART
#[cfg(any(
    feature = "gd32f130x4",
    feature = "gd32f150x4",
    feature = "gd32f170x4",
    feature = "gd32f190x4"
))]
mod pins {
    use super::*;

    impl TxPin<Usart0> for PA2<Alternate<AF1>> {}
    impl RxPin<Usart0> for PA3<Alternate<AF1>> {}
    impl TxPin<Usart0> for PA14<Alternate<AF1>> {}
    impl RxPin<Usart0> for PA15<Alternate<AF1>> {}
}

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
            .ctl0()
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

impl<USART: RcuBus + Enable + Reset + Deref<Target = usart0::RegisterBlock>, TXPIN: TxPin<USART>>
    Serial<USART, TXPIN, ()>
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
        usart
            .ctl0()
            .modify(|_, w| w.ten().enabled().uen().enabled());

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

impl<USART: RcuBus + Enable + Reset + Deref<Target = usart0::RegisterBlock>, RXPIN: RxPin<USART>>
    Serial<USART, (), RXPIN>
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
        usart
            .ctl0()
            .modify(|_, w| w.ren().enabled().uen().enabled());

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
            Event::Rbne => self.usart.ctl0().modify(|_, w| w.rbneie().enabled()),
            Event::Tbe => self.usart.ctl0().modify(|_, w| w.tbeie().enabled()),
            Event::Idle => self.usart.ctl0().modify(|_, w| w.idleie().enabled()),
        }
    }

    /// Disable an interrupt event.
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Rbne => self.usart.ctl0().modify(|_, w| w.rbneie().disabled()),
            Event::Tbe => self.usart.ctl0().modify(|_, w| w.tbeie().disabled()),
            Event::Idle => self.usart.ctl0().modify(|_, w| w.idleie().disabled()),
        }
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>> Rx<USART> {
    /// Enable the RBNE interrupt.
    pub fn listen(&mut self) {
        unsafe { &*self.usart }
            .ctl0()
            .modify(|_, w| w.rbneie().enabled());
    }

    /// Disable the RBNE interrupt.
    pub fn unlisten(&mut self) {
        unsafe { &*self.usart }
            .ctl0()
            .modify(|_, w| w.rbneie().disabled());
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>> Tx<USART> {
    /// Enable the TBE interrupt.
    pub fn listen(&mut self) {
        unsafe { &*self.usart }
            .ctl0()
            .modify(|_, w| w.tbeie().enabled());
    }

    /// Disable the TBE interrupt.
    pub fn unlisten(&mut self) {
        unsafe { &*self.usart }
            .ctl0()
            .modify(|_, w| w.tbeie().disabled());
    }
}

// Implement writing traits if the USART has a TX pin assigned.
impl<USART: Deref<Target = usart0::RegisterBlock>, TXPIN: TxPin<USART>, RXPIN> Write
    for Serial<USART, TXPIN, RXPIN>
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.usart.write_buffer(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        nb::block!(self.usart.flush())?;
        Ok(())
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>, TXPIN: TxPin<USART>, RXPIN> WriteReady
    for Serial<USART, TXPIN, RXPIN>
{
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.usart.write_ready())
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<USART: Deref<Target = usart0::RegisterBlock>, TXPIN: TxPin<USART>, RXPIN>
    embedded_hal_02::serial::Write<u8> for Serial<USART, TXPIN, RXPIN>
{
    type Error = Infallible;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.usart.flush()
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        self.usart.write(byte)
    }
}

impl<USART> ErrorType for Tx<USART> {
    type Error = Error;
}

impl<USART: Deref<Target = usart0::RegisterBlock>> Write for Tx<USART> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        unsafe { &*self.usart }.write_buffer(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        nb::block!(unsafe { &*self.usart }.flush())?;
        Ok(())
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>> WriteReady for Tx<USART> {
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(unsafe { &*self.usart }.write_ready())
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<USART: Deref<Target = usart0::RegisterBlock>> embedded_hal_02::serial::Write<u8>
    for Tx<USART>
{
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
        self.write_all(s.as_bytes()).map_err(|_| core::fmt::Error)
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>> fmt::Write for Tx<USART> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.write_all(s.as_bytes()).map_err(|_| core::fmt::Error)
    }
}

impl<USART, TXPIN, RXPIN> ErrorType for Serial<USART, TXPIN, RXPIN> {
    type Error = Error;
}

// Implement reading traits if the USART has an RX pin assigned.
impl<USART: Deref<Target = usart0::RegisterBlock>, TXPIN, RXPIN: RxPin<USART>> Read
    for Serial<USART, TXPIN, RXPIN>
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.usart.read_buffer(buf)
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>, TXPIN, RXPIN: RxPin<USART>> ReadReady
    for Serial<USART, TXPIN, RXPIN>
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.usart.read_ready())
    }
}

impl<USART> ErrorType for Rx<USART> {
    type Error = Error;
}

impl<USART: Deref<Target = usart0::RegisterBlock>> Read for Rx<USART> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        unsafe { &*self.usart }.read_buffer(buf)
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>> ReadReady for Rx<USART> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(unsafe { &*self.usart }.read_ready())
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<USART: Deref<Target = usart0::RegisterBlock>, TXPIN, RXPIN: RxPin<USART>>
    embedded_hal_02::serial::Read<u8> for Serial<USART, TXPIN, RXPIN>
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        self.usart.read()
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<USART: Deref<Target = usart0::RegisterBlock>> embedded_hal_02::serial::Read<u8> for Rx<USART> {
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
        assert!((16..=0xFFFF).contains(&baud_rate_ratio));
        self.baud().write(|w| unsafe { w.bits(baud_rate_ratio) });

        // Configure parity. Note that the parity bit counts towards the word length, so we have to
        // increase it to 9 bits if parity is enabled so as to still get 8 data bits.
        match config.parity {
            Parity::ParityNone => {
                self.ctl0().modify(|_, w| w.pcen().disabled().wl().bit8());
            }
            Parity::ParityEven => {
                self.ctl0()
                    .modify(|_, w| w.pcen().enabled().wl().bit9().pm().even());
            }
            Parity::ParityOdd => {
                self.ctl0()
                    .modify(|_, w| w.pcen().enabled().wl().bit9().pm().odd());
            }
        }

        // Configure stop bits.
        self.ctl1().modify(|_, w| w.stb().variant(config.stopbits));
    }
}

trait UsartReadWrite {
    fn read(&mut self) -> nb::Result<u8, Error>;
    fn read_ready(&self) -> bool;
    fn flush(&mut self) -> nb::Result<(), Infallible>;
    fn write(&mut self, byte: u8) -> nb::Result<(), Infallible>;
    fn write_ready(&self) -> bool;

    /// Reads into the given buffer, blocking until at least one byte is available.
    fn read_buffer(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        let mut bytes_read = 0;
        while bytes_read < buf.len() {
            match self.read() {
                Ok(b) => {
                    buf[bytes_read] = b;
                    bytes_read += 1;
                }
                Err(nb::Error::WouldBlock) => {
                    if bytes_read > 0 {
                        break;
                    }
                    spin_loop();
                }
                Err(nb::Error::Other(e)) => {
                    return Err(e);
                }
            }
        }
        Ok(bytes_read)
    }

    /// Writes at least one byte from the given buffer, blocking if necessary.
    fn write_buffer(&mut self, buf: &[u8]) -> Result<usize, Error> {
        let mut bytes_written = 0;
        while bytes_written < buf.len() {
            match self.write(buf[bytes_written]) {
                Ok(()) => {
                    bytes_written += 1;
                }
                Err(nb::Error::WouldBlock) => {
                    if bytes_written > 0 {
                        break;
                    }
                    spin_loop();
                }
                Err(nb::Error::Other(_)) => {
                    unreachable!()
                }
            }
        }
        Ok(bytes_written)
    }
}

impl<USART: Deref<Target = usart0::RegisterBlock>> UsartReadWrite for USART {
    fn read(&mut self) -> nb::Result<u8, Error> {
        let status = self.stat().read();

        if status.perr().bit_is_set() {
            self.intc().write(|w| w.pec().clear());
            Err(nb::Error::Other(Error::Parity))
        } else if status.ferr().bit_is_set() {
            self.intc().write(|w| w.fec().clear());
            Err(nb::Error::Other(Error::Framing))
        } else if status.nerr().bit_is_set() {
            self.intc().write(|w| w.nec().clear());
            Err(nb::Error::Other(Error::Noise))
        } else if status.orerr().bit_is_set() {
            self.intc().write(|w| w.orec().clear());
            // Discard the previous received byte.
            self.cmd().write(|w| w.rxfcmd().discard());
            Err(nb::Error::Other(Error::Overrun))
        } else if status.rbne().bit_is_set() {
            Ok(self.rdata().read().rdata().bits() as u8)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn read_ready(&self) -> bool {
        self.stat().read().rbne().bit_is_set()
    }

    fn flush(&mut self) -> nb::Result<(), Infallible> {
        let status = self.stat().read();
        if status.tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Infallible> {
        let status = self.stat().read();
        if status.tbe().bit_is_set() {
            self.tdata()
                .write(|w| unsafe { w.tdata().bits(byte.into()) });
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write_ready(&self) -> bool {
        self.stat().read().tbe().bit_is_set()
    }
}

macro_rules! serialdma {
    ($(
        $USARTX:ty: (
            $RxDmaX:ident,
            $TxDmaX:ident,
            $dmarxch:ty,
            $dmatxch:ty,
        ),
    )+) => {
        $(
            pub type $RxDmaX = RxDma<Rx<$USARTX>, $dmarxch>;
            pub type $TxDmaX = TxDma<Tx<$USARTX>, $dmatxch>;

            impl Receive for $RxDmaX {
                type RxChannel = $dmarxch;
                type TransmittedWord = u8;
            }

            impl Transmit for $TxDmaX {
                type TxChannel = $dmatxch;
                type ReceivedWord = u8;
            }

            impl TransferPayload for $RxDmaX {
                fn start(&mut self) {
                    self.channel.start();
                }

                fn stop(&mut self) {
                    self.channel.stop();
                }
            }

            impl TransferPayload for $TxDmaX {
                fn start(&mut self) {
                    self.channel.start();
                }

                fn stop(&mut self) {
                    self.channel.stop();
                }
            }

            impl Rx<$USARTX> {
                pub fn with_dma(self, channel: $dmarxch) -> $RxDmaX {
                    RxDma {
                        payload: self,
                        channel,
                    }
                }
            }

            impl Tx<$USARTX> {
                pub fn with_dma(self, channel: $dmatxch) -> $TxDmaX {
                    TxDma {
                        payload: self,
                        channel,
                    }
                }
            }

            impl $RxDmaX {
                pub fn split(mut self) -> (Rx<$USARTX>, $dmarxch) {
                    self.stop();
                    let RxDma { payload, channel } = self;
                    (payload, channel)
                }
            }

            impl $TxDmaX {
                pub fn split(mut self) -> (Tx<$USARTX>, $dmatxch) {
                    self.stop();
                    let TxDma { payload, channel } = self;
                    (payload, channel)
                }
            }

            impl<B> CircReadDma<B, u8> for $RxDmaX
            where
                &'static mut [B; 2]: WriteBuffer<Word = u8>,
                B: 'static,
            {
                fn circ_read(mut self, mut buffer: &'static mut [B; 2]) -> CircBuffer<B, Self> {
                    // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                    // until the end of the transfer.
                    let (ptr, len) = unsafe { buffer.write_buffer() };
                    self.channel
                        .set_peripheral_address(unsafe { &(*<$USARTX>::ptr()).rdata() as *const _ as u32 }, false);
                    self.channel.set_memory_address(ptr as u32, true);
                    self.channel.set_transfer_length(len);

                    atomic::compiler_fence(Ordering::Release);

                    self.channel
                        .configure_from_peripheral(Priority::Medium, Width::Bits8, Width::Bits8, true);

                    self.start();

                    CircBuffer::new(buffer, self)
                }
            }

            impl<B> ReadDma<B, u8> for $RxDmaX
            where
                B: WriteBuffer<Word = u8>,
            {
                fn read(mut self, mut buffer: B) -> Transfer<W, B, Self> {
                    // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                    // until the end of the transfer.
                    let (ptr, len) = unsafe { buffer.write_buffer() };
                    self.channel
                        .set_peripheral_address(unsafe { &(*<$USARTX>::ptr()).rdata() as *const _ as u32 }, false);
                    self.channel.set_memory_address(ptr as u32, true);
                    self.channel.set_transfer_length(len);

                    atomic::compiler_fence(Ordering::Release);
                    self.channel
                        .configure_from_peripheral(Priority::Medium, Width::Bits8, Width::Bits8, false);
                    self.start();

                    Transfer::w(buffer, self)
                }
            }

            impl<B> WriteDma<B, u8> for $TxDmaX
            where
                B: ReadBuffer<Word = u8>,
            {
                fn write(mut self, buffer: B) -> Transfer<R, B, Self> {
                    // Clear transmission complete bit.
                    unsafe { &*self.payload.usart }
                        .intc()
                        .write(|w| w.tcc().clear());

                    // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                    // until the end of the transfer.
                    let (ptr, len) = unsafe { buffer.read_buffer() };

                    self.channel
                        .set_peripheral_address(unsafe { &(*<$USARTX>::ptr()).tdata() as *const _ as u32 }, false);

                    self.channel.set_memory_address(ptr as u32, true);
                    self.channel.set_transfer_length(len);

                    atomic::compiler_fence(Ordering::Release);

                    self.channel
                        .configure_to_peripheral(Priority::Medium, Width::Bits8, Width::Bits8, false);
                    self.start();

                    Transfer::r(buffer, self)
                }
            }
        )+
    }
}

serialdma! {
    pac::Usart0: (
        RxDma0,
        TxDma0,
        dma::C2,
        dma::C1,
    ),
}

#[cfg(any(
    feature = "gd32f130x6",
    feature = "gd32f130x8",
    feature = "gd32f150x6",
    feature = "gd32f150x8",
    feature = "gd32f170x6",
    feature = "gd32f170x8",
    feature = "gd32f190x6",
    feature = "gd32f190x8",
))]
serialdma! {
    pac::Usart1: (
        RxDma1,
        TxDma1,
        dma::C4,
        dma::C3,
    ),
}
