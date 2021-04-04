// Copyright 2021 The gd32f1x0-hal authors.
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! # Direct Memory Access

use crate::pac::{dma, DMA};
use crate::rcu::{Enable, AHB};
use core::{
    marker::PhantomData,
    mem, ptr,
    sync::atomic::{self, compiler_fence, Ordering},
};
use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};

#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    Overrun,
}

pub enum Event {
    HalfTransfer,
    TransferComplete,
}

#[derive(Clone, Copy, PartialEq)]
pub enum Half {
    First,
    Second,
}

pub struct CircBuffer<BUFFER, PAYLOAD>
where
    BUFFER: 'static,
{
    buffer: &'static mut [BUFFER; 2],
    payload: PAYLOAD,
    readable_half: Half,
}

impl<BUFFER, PAYLOAD> CircBuffer<BUFFER, PAYLOAD>
where
    &'static mut [BUFFER; 2]: StaticWriteBuffer,
    BUFFER: 'static,
{
    pub(crate) fn new(buf: &'static mut [BUFFER; 2], payload: PAYLOAD) -> Self {
        CircBuffer {
            buffer: buf,
            payload,
            readable_half: Half::Second,
        }
    }
}

pub trait DmaExt {
    type Channels;

    fn split(self, ahb: &mut AHB) -> Self::Channels;
}

pub trait TransferPayload {
    fn start(&mut self);
    fn stop(&mut self);
}

pub struct Transfer<MODE, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    _mode: PhantomData<MODE>,
    buffer: BUFFER,
    payload: PAYLOAD,
}

impl<BUFFER, PAYLOAD> Transfer<R, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    pub(crate) fn r(buffer: BUFFER, payload: PAYLOAD) -> Self {
        Transfer {
            _mode: PhantomData,
            buffer,
            payload,
        }
    }
}

impl<BUFFER, PAYLOAD> Transfer<W, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    pub(crate) fn w(buffer: BUFFER, payload: PAYLOAD) -> Self {
        Transfer {
            _mode: PhantomData,
            buffer,
            payload,
        }
    }
}

impl<MODE, BUFFER, PAYLOAD> Drop for Transfer<MODE, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    fn drop(&mut self) {
        self.payload.stop();
        compiler_fence(Ordering::SeqCst);
    }
}

/// Read transfer
pub struct R;

/// Write transfer
pub struct W;

#[allow(dead_code)]
#[derive(Copy, Clone, Debug)]
pub(crate) enum Priority {
    Low = 0,
    Medium = 1,
    High = 2,
    VeryHigh = 3,
}

#[allow(dead_code)]
#[derive(Copy, Clone, Debug)]
pub(crate) enum Width {
    Bits8 = 0,
    Bits16 = 1,
    Bits32 = 2,
}

macro_rules! dma {
    {$($CX:ident: (
            $htfifX:ident,
            $ftfifX:ident,
            $htfifcX:ident,
            $ftfifcX:ident,
            $gicX:ident,
            $chXctl:ident,
            $chXcnt:ident,
            $chXmaddr:ident,
            $chXpaddr:ident,
    ),)+} => {
        pub struct Channels($(pub $CX),+);

        $(
            /// A singleton that represents a single DMA channel (channel X in this case)
            ///
            /// This singleton has exclusive access to the registers of the DMA channel X
            pub struct $CX {
                _0: (),
            }

            impl $CX {
                /// Associated peripheral `address`
                ///
                /// `inc` indicates whether the address will be incremented after every byte transfer
                pub fn set_peripheral_address(&mut self, address: u32, inc: bool) {
                    unsafe { &(*DMA::ptr()).$chXpaddr }.write(|w| unsafe { w.paddr().bits(address) });
                    unsafe { &(*DMA::ptr()).$chXctl }.modify(|_, w| w.pnaga().bit(inc));
                }

                /// `address` where from/to data will be read/write
                ///
                /// `inc` indicates whether the address will be incremented after every byte transfer
                pub fn set_memory_address(&mut self, address: u32, inc: bool) {
                    unsafe { &(*DMA::ptr()).$chXmaddr }.write(|w| unsafe { w.maddr().bits(address) });
                    unsafe { &(*DMA::ptr()).$chXctl }.modify(|_, w| w.mnaga().bit(inc));
                }

                /// Number of bytes to transfer
                pub fn set_transfer_length(&mut self, len: usize) {
                    unsafe { &(*DMA::ptr()).$chXcnt }
                        .write(|w| w.cnt().bits(cast::u16(len).unwrap()));
                }

                /// Starts the DMA transfer
                pub fn start(&mut self) {
                    unsafe { &(*DMA::ptr()).$chXctl }.modify(|_, w| w.chen().enabled());
                }

                /// Stops the DMA transfer
                pub fn stop(&mut self) {
                    self.intc().write(|w| w.$gicX().clear());
                    unsafe { &(*DMA::ptr()).$chXctl }.modify(|_, w| w.chen().disabled());
                }

                /// Returns `true` if there's a transfer in progress
                pub fn in_progress(&self) -> bool {
                    self.intf().$ftfifX().is_not_complete()
                }

                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::HalfTransfer => unsafe { &(*DMA::ptr()).$chXctl }.modify(|_, w| w.htfie().enabled()),
                        Event::TransferComplete => unsafe { &(*DMA::ptr()).$chXctl }.modify(|_, w| w.ftfie().enabled()),
                    }
                }

                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::HalfTransfer => unsafe { &(*DMA::ptr()).$chXctl }.modify(|_, w| w.htfie().disabled()),
                        Event::TransferComplete => unsafe { &(*DMA::ptr()).$chXctl }.modify(|_, w| w.ftfie().disabled()),
                    }
                }

                /// Configures the DMA channel to transfer data from a peripheral to memory.
                #[allow(dead_code)]
                pub(crate) fn configure_from_peripheral(
                    &mut self,
                    priority: Priority,
                    memory_width: Width,
                    peripheral_width: Width,
                    circular: bool,
                ) {
                    unsafe {
                        (*DMA::ptr()).$chXctl.modify(|_, w| {
                            w.m2m()
                                .disabled()
                                .dir()
                                .from_peripheral()
                                .prio()
                                .bits(priority as u8)
                                .mwidth()
                                .bits(memory_width as u8)
                                .pwidth()
                                .bits(peripheral_width as u8)
                                .cmen()
                                .bit(circular)
                        });
                    }
                }

                fn intf(&self) -> dma::intf::R {
                    // NOTE(unsafe) atomic read with no side effects
                    unsafe { (*DMA::ptr()).intf.read() }
                }

                fn intc(&self) -> &dma::INTC {
                    unsafe { &(*DMA::ptr()).intc }
                }

                fn get_cnt(&self) -> u16 {
                    unsafe { &(*DMA::ptr()).$chXcnt }.read().cnt().bits()
                }
            }

            impl<B, PAYLOAD> CircBuffer<B, RxDma<PAYLOAD, $CX>>
            where
                RxDma<PAYLOAD, $CX>: TransferPayload,
            {
                /// Peeks into the readable half of the buffer
                pub fn peek<R, F>(&mut self, f: F) -> Result<R, Error>
                where
                    F: FnOnce(&B, Half) -> R,
                {
                    let half_being_read = self.readable_half()?;

                    let buf = match half_being_read {
                        Half::First => &self.buffer[0],
                        Half::Second => &self.buffer[1],
                    };

                    // XXX does this need a compiler barrier?
                    let ret = f(buf, half_being_read);

                    let intf = self.payload.channel.intf();
                    let first_half_is_done = intf.$htfifX().is_half();
                    let second_half_is_done = intf.$ftfifX().is_complete();

                    if (half_being_read == Half::First && second_half_is_done)
                        || (half_being_read == Half::Second && first_half_is_done)
                    {
                        Err(Error::Overrun)
                    } else {
                        Ok(ret)
                    }
                }

                /// Returns the `Half` of the buffer that can be read
                pub fn readable_half(&mut self) -> Result<Half, Error> {
                    let intf = self.payload.channel.intf();
                    let first_half_is_done = intf.$htfifX().is_half();
                    let second_half_is_done = intf.$ftfifX().is_complete();

                    if first_half_is_done && second_half_is_done {
                        return Err(Error::Overrun);
                    }

                    let last_read_half = self.readable_half;

                    Ok(match last_read_half {
                        Half::First => {
                            if second_half_is_done {
                                self.payload.channel.intc().write(|w| w.$ftfifcX().clear());

                                self.readable_half = Half::Second;
                                Half::Second
                            } else {
                                last_read_half
                            }
                        }
                        Half::Second => {
                            if first_half_is_done {
                                self.payload.channel.intc().write(|w| w.$htfifcX().clear());

                                self.readable_half = Half::First;
                                Half::First
                            } else {
                                last_read_half
                            }
                        }
                    })
                }

                /// Stops the transfer and returns the underlying buffer and RxDma
                pub fn stop(mut self) -> (&'static mut [B; 2], RxDma<PAYLOAD, $CX>) {
                    self.payload.stop();

                    (self.buffer, self.payload)
                }
            }

            impl<BUFFER, PAYLOAD, MODE> Transfer<MODE, BUFFER, RxDma<PAYLOAD, $CX>>
            where
                RxDma<PAYLOAD, $CX>: TransferPayload,
            {
                pub fn is_done(&self) -> bool {
                    !self.payload.channel.in_progress()
                }

                pub fn wait(mut self) -> (BUFFER, RxDma<PAYLOAD, $CX>) {
                    while !self.is_done() {}

                    atomic::compiler_fence(Ordering::Acquire);

                    self.payload.stop();

                    // we need a read here to make the Acquire fence effective
                    // we do *not* need this if `dma.stop` does a RMW operation
                    unsafe {
                        ptr::read_volatile(&0);
                    }

                    // we need a fence here for the same reason we need one in `Transfer.wait`
                    atomic::compiler_fence(Ordering::Acquire);

                    // `Transfer` needs to have a `Drop` implementation, because we accept
                    // managed buffers that can free their memory on drop. Because of that
                    // we can't move out of the `Transfer`'s fields, so we use `ptr::read`
                    // and `mem::forget`.
                    //
                    // NOTE(unsafe) There is no panic branch between getting the resources
                    // and forgetting `self`.
                    unsafe {
                        let buffer = ptr::read(&self.buffer);
                        let payload = ptr::read(&self.payload);
                        mem::forget(self);
                        (buffer, payload)
                    }
                }
            }

            impl<BUFFER, PAYLOAD, MODE> Transfer<MODE, BUFFER, TxDma<PAYLOAD, $CX>>
            where
                TxDma<PAYLOAD, $CX>: TransferPayload,
            {
                pub fn is_done(&self) -> bool {
                    !self.payload.channel.in_progress()
                }

                pub fn wait(mut self) -> (BUFFER, TxDma<PAYLOAD, $CX>) {
                    while !self.is_done() {}

                    atomic::compiler_fence(Ordering::Acquire);

                    self.payload.stop();

                    // we need a read here to make the Acquire fence effective
                    // we do *not* need this if `dma.stop` does a RMW operation
                    unsafe {
                        ptr::read_volatile(&0);
                    }

                    // we need a fence here for the same reason we need one in `Transfer.wait`
                    atomic::compiler_fence(Ordering::Acquire);

                    // `Transfer` needs to have a `Drop` implementation, because we accept
                    // managed buffers that can free their memory on drop. Because of that
                    // we can't move out of the `Transfer`'s fields, so we use `ptr::read`
                    // and `mem::forget`.
                    //
                    // NOTE(unsafe) There is no panic branch between getting the resources
                    // and forgetting `self`.
                    unsafe {
                        let buffer = ptr::read(&self.buffer);
                        let payload = ptr::read(&self.payload);
                        mem::forget(self);
                        (buffer, payload)
                    }
                }
            }

            impl<BUFFER, PAYLOAD> Transfer<W, BUFFER, RxDma<PAYLOAD, $CX>>
            where
                RxDma<PAYLOAD, $CX>: TransferPayload,
            {
                pub fn peek<T>(&self) -> &[T]
                where
                    BUFFER: AsRef<[T]>,
                {
                    let pending = self.payload.channel.get_cnt() as usize;

                    let slice = self.buffer.as_ref();
                    let capacity = slice.len();

                    &slice[..(capacity - pending)]
                }
            }
        )+

        impl DmaExt for DMA {
            type Channels = Channels;

            fn split(self, ahb: &mut AHB) -> Channels {
                DMA::enable(ahb);

                // reset the DMA control registers (stops all on-going transfers)
                $(
                    self.$chXctl.reset();
                )+

                Channels($($CX { _0: () }),+)
            }
        }
    }
}

/// DMA Receiver
pub struct RxDma<PAYLOAD, RXCH> {
    pub(crate) payload: PAYLOAD,
    pub channel: RXCH,
}

/// DMA Transmitter
pub struct TxDma<PAYLOAD, TXCH> {
    pub(crate) payload: PAYLOAD,
    pub channel: TXCH,
}

/// DMA Receiver/Transmitter
pub struct RxTxDma<PAYLOAD, RXCH, TXCH> {
    pub(crate) payload: PAYLOAD,
    pub rxchannel: RXCH,
    pub txchannel: TXCH,
}

pub trait Receive {
    type RxChannel;
    type TransmittedWord;
}

pub trait Transmit {
    type TxChannel;
    type ReceivedWord;
}

/// Trait for circular DMA readings from peripheral to memory.
pub trait CircReadDma<B, RS>: Receive
where
    &'static mut [B; 2]: StaticWriteBuffer<Word = RS>,
    B: 'static,
    Self: core::marker::Sized,
{
    fn circ_read(self, buffer: &'static mut [B; 2]) -> CircBuffer<B, Self>;
}

/// Trait for DMA readings from peripheral to memory.
pub trait ReadDma<B, RS>: Receive
where
    B: StaticWriteBuffer<Word = RS>,
    Self: core::marker::Sized + TransferPayload,
{
    fn read(self, buffer: B) -> Transfer<W, B, Self>;
}

/// Trait for DMA writing from memory to peripheral.
pub trait WriteDma<B, TS>: Transmit
where
    B: StaticReadBuffer<Word = TS>,
    Self: core::marker::Sized + TransferPayload,
{
    fn write(self, buffer: B) -> Transfer<R, B, Self>;
}

dma! {
    C0: (
        htfif0, ftfif0,
        htfifc0, ftfifc0, gic0,
        ch0ctl0, ch0cnt, ch0maddr, ch0paddr,
    ),
    C1: (
        htfif1, ftfif1,
        htfifc1, ftfifc1, gic1,
        ch1ctl0, ch1cnt, ch1maddr, ch1paddr,
    ),
    C2: (
        htfif2, ftfif2,
        htfifc2, ftfifc2, gic2,
        ch2ctl0, ch2cnt, ch2maddr, ch2paddr,
    ),
    C3: (
        htfif3, ftfif3,
        htfifc3, ftfifc3, gic3,
        ch3ctl0, ch3cnt, ch3maddr, ch3paddr,
    ),
    C4: (
        htfif4, ftfif4,
        htfifc4, ftfifc4, gic4,
        ch4ctl0, ch4cnt, ch4maddr, ch4paddr,
    ),
    C5: (
        htfif5, ftfif5,
        htfifc5, ftfifc5, gic5,
        ch5ctl0, ch5cnt, ch5maddr, ch5paddr,
    ),
    C6: (
        htfif6, ftfif6,
        htfifc6, ftfifc6, gic6,
        ch6ctl0, ch6cnt, ch6maddr, ch6paddr,
    ),
}
