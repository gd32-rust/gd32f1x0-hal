// Copyright 2021 The gd32f1x0-hal authors.
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! # Serial Communication (USART)
//!
//! This module contains the functions to utilize the USART (Universal
//! synchronous asynchronous receiver transmitter).

use crate::dma::{
    self, CircBuffer, CircReadDma, Priority, ReadDma, Receive, RxDma, Transfer, TransferPayload,
    Transmit, TxDma, Width, WriteDma, R, W,
};
use crate::gpio::gpioa::{PA10, PA14, PA15, PA2, PA3, PA9};
use crate::gpio::gpiob::{PB6, PB7};
use crate::gpio::{Alternate, AF0, AF1};
use crate::pac::{self, usart0, usart0::ctl1::STB_A, USART0};
use crate::rcu::{sealed::RcuBus, Clocks, Enable, GetBusFreq, Reset};
use crate::time::{Bps, U32Ext};
use core::convert::Infallible;
use core::fmt;
use core::marker::PhantomData;
use core::ops::Deref;
use core::sync::atomic::{self, Ordering};
use embedded_dma::{ReadBuffer, WriteBuffer};
use embedded_hal::serial::{Read, Write};

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
impl RxPin<USART0> for PA10<Alternate<AF1>> {}
impl TxPin<USART0> for PB6<Alternate<AF0>> {}
impl RxPin<USART0> for PB7<Alternate<AF0>> {}

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
    use crate::gpio::{gpioa::PA8, AF4};
    use crate::pac::USART1;

    impl TxPin<USART1> for PA2<Alternate<AF1>> {}
    impl RxPin<USART1> for PA3<Alternate<AF1>> {}
    impl TxPin<USART1> for PA8<Alternate<AF4>> {}
    impl RxPin<USART1> for PB0<Alternate<AF4>> {}
    impl TxPin<USART1> for PA14<Alternate<AF1>> {}
    impl RxPin<USART1> for PA15<Alternate<AF1>> {}
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

    impl TxPin<USART0> for PA2<Alternate<AF1>> {}
    impl RxPin<USART0> for PA3<Alternate<AF1>> {}
    impl TxPin<USART0> for PA14<Alternate<AF1>> {}
    impl RxPin<USART0> for PA15<Alternate<AF1>> {}
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
        assert!((16..=0xFFFF).contains(&baud_rate_ratio));
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
            // Discard the previous received byte.
            self.cmd.write(|w| w.rxfcmd().discard());
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
                        .set_peripheral_address(unsafe { &(*<$USARTX>::ptr()).rdata as *const _ as u32 }, false);
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
                        .set_peripheral_address(unsafe { &(*<$USARTX>::ptr()).rdata as *const _ as u32 }, false);
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
                        .intc
                        .write(|w| w.tcc().clear());

                    // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                    // until the end of the transfer.
                    let (ptr, len) = unsafe { buffer.read_buffer() };

                    self.channel
                        .set_peripheral_address(unsafe { &(*<$USARTX>::ptr()).tdata as *const _ as u32 }, false);

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
    pac::USART0: (
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
    pac::USART1: (
        RxDma1,
        TxDma1,
        dma::C4,
        dma::C3,
    ),
}
