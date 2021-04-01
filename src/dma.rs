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

#[allow(clippy::manual_non_exhaustive)]
pub struct Channels((), pub C0);

/// A singleton that represents a single DMA channel (channel X in this case)
///
/// This singleton has exclusive access to the registers of the DMA channel X
pub struct C0 {
    _0: (),
}

impl C0 {
    /// Associated peripheral `address`
    ///
    /// `inc` indicates whether the address will be incremented after every byte transfer
    pub fn set_peripheral_address(&mut self, address: u32, inc: bool) {
        self.chpaddr().write(|w| unsafe { w.paddr().bits(address) });
        self.chctl().modify(|_, w| w.pnaga().bit(inc));
    }

    /// `address` where from/to data will be read/write
    ///
    /// `inc` indicates whether the address will be incremented after every byte transfer
    pub fn set_memory_address(&mut self, address: u32, inc: bool) {
        self.chmaddr().write(|w| unsafe { w.maddr().bits(address) });
        self.chctl().modify(|_, w| w.mnaga().bit(inc));
    }

    /// Number of bytes to transfer
    pub fn set_transfer_length(&mut self, len: usize) {
        self.chcnt()
            .write(|w| w.cnt().bits(cast::u16(len).unwrap()));
    }

    /// Starts the DMA transfer
    pub fn start(&mut self) {
        self.chctl().modify(|_, w| w.chen().enabled());
    }

    /// Stops the DMA transfer
    pub fn stop(&mut self) {
        self.ifcr().write(|w| w.gic0().clear());
        self.chctl().modify(|_, w| w.chen().disabled());
    }

    /// Returns `true` if there's a transfer in progress
    pub fn in_progress(&self) -> bool {
        self.isr().ftfif0().is_not_complete()
    }

    pub fn listen(&mut self, event: Event) {
        match event {
            Event::HalfTransfer => self.chctl().modify(|_, w| w.htfie().enabled()),
            Event::TransferComplete => self.chctl().modify(|_, w| w.ftfie().enabled()),
        }
    }

    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::HalfTransfer => self.chctl().modify(|_, w| w.htfie().disabled()),
            Event::TransferComplete => self.chctl().modify(|_, w| w.ftfie().disabled()),
        }
    }

    fn chctl(&mut self) -> &dma::CH0CTL0 {
        unsafe { &(*DMA::ptr()).ch0ctl0 }
    }

    fn chcnt(&mut self) -> &dma::CH0CNT {
        unsafe { &(*DMA::ptr()).ch0cnt }
    }

    fn chmaddr(&mut self) -> &dma::CH0MADDR {
        unsafe { &(*DMA::ptr()).ch0maddr }
    }

    fn chpaddr(&mut self) -> &dma::CH0PADDR {
        unsafe { &(*DMA::ptr()).ch0paddr }
    }

    fn isr(&self) -> dma::intf::R {
        // NOTE(unsafe) atomic read with no side effects
        unsafe { (*DMA::ptr()).intf.read() }
    }

    fn ifcr(&self) -> &dma::INTC {
        unsafe { &(*DMA::ptr()).intc }
    }

    fn get_ndtr(&self) -> u16 {
        // NOTE(unsafe) atomic read with no side effects
        unsafe { &(*DMA::ptr()) }.ch0cnt.read().cnt().bits()
    }
}

impl<B, PAYLOAD> CircBuffer<B, RxDma<PAYLOAD, C0>>
where
    RxDma<PAYLOAD, C0>: TransferPayload,
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

        let isr = self.payload.channel.isr();
        let first_half_is_done = isr.htfif0().is_half();
        let second_half_is_done = isr.ftfif0().is_complete();

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
        let isr = self.payload.channel.isr();
        let first_half_is_done = isr.htfif0().is_half();
        let second_half_is_done = isr.ftfif0().is_complete();

        if first_half_is_done && second_half_is_done {
            return Err(Error::Overrun);
        }

        let last_read_half = self.readable_half;

        Ok(match last_read_half {
            Half::First => {
                if second_half_is_done {
                    self.payload.channel.ifcr().write(|w| w.ftfifc0().clear());

                    self.readable_half = Half::Second;
                    Half::Second
                } else {
                    last_read_half
                }
            }
            Half::Second => {
                if first_half_is_done {
                    self.payload.channel.ifcr().write(|w| w.htfifc0().clear());

                    self.readable_half = Half::First;
                    Half::First
                } else {
                    last_read_half
                }
            }
        })
    }

    /// Stops the transfer and returns the underlying buffer and RxDma
    pub fn stop(mut self) -> (&'static mut [B; 2], RxDma<PAYLOAD, C0>) {
        self.payload.stop();

        (self.buffer, self.payload)
    }
}

impl<BUFFER, PAYLOAD, MODE> Transfer<MODE, BUFFER, RxDma<PAYLOAD, C0>>
where
    RxDma<PAYLOAD, C0>: TransferPayload,
{
    pub fn is_done(&self) -> bool {
        !self.payload.channel.in_progress()
    }

    pub fn wait(mut self) -> (BUFFER, RxDma<PAYLOAD, C0>) {
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

impl<BUFFER, PAYLOAD, MODE> Transfer<MODE, BUFFER, TxDma<PAYLOAD, C0>>
where
    TxDma<PAYLOAD, C0>: TransferPayload,
{
    pub fn is_done(&self) -> bool {
        !self.payload.channel.in_progress()
    }

    pub fn wait(mut self) -> (BUFFER, TxDma<PAYLOAD, C0>) {
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

impl<BUFFER, PAYLOAD> Transfer<W, BUFFER, RxDma<PAYLOAD, C0>>
where
    RxDma<PAYLOAD, C0>: TransferPayload,
{
    pub fn peek<T>(&self) -> &[T]
    where
        BUFFER: AsRef<[T]>,
    {
        let pending = self.payload.channel.get_ndtr() as usize;

        let slice = self.buffer.as_ref();
        let capacity = slice.len();

        &slice[..(capacity - pending)]
    }
}

impl DmaExt for DMA {
    type Channels = Channels;

    fn split(self, ahb: &mut AHB) -> Channels {
        DMA::enable(ahb);

        // reset the DMA control registers (stops all on-going transfers)

        self.ch0ctl0.reset();

        Channels((), C0 { _0: () })
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
