// Copyright 2021 The gd32f1x0-hal authors.
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! # API for the Analog to Digital converter

use crate::dma::{
    CircBuffer, CircReadDma, Priority, ReadDma, Receive, RxDma, Transfer, TransferPayload, Width,
    C0, W,
};
use crate::gpio::Analog;
use crate::gpio::{gpioa, gpiob, gpioc};
use crate::pac::ADC;
use crate::rcu::{Clocks, Enable, Reset, APB2};
use core::{
    convert::Infallible,
    marker::PhantomData,
    sync::atomic::{self, Ordering},
};
use cortex_m::asm::delay;
use embedded_dma::StaticWriteBuffer;
use embedded_hal::adc::{Channel, OneShot};
use gd32f1::gd32f1x0::adc::{
    ctl1::{CTN_A, DAL_A},
    sampt0::SPT10_A,
    sampt1::SPT0_A,
};

/// The number of ADC clock cycles to wait between powering on and starting calibration.
const ADC_CALIBRATION_CYCLES: u32 = 14;

/// Typical Vtemp at 25°C, in mV.
const VTEMP_25: u16 = 1430;

/// Typical dV/°C for Vtemp.
const VTEMP_SLOPE: u16 = 43;

/// The reference voltage in mV, i.e. 1.2 V.
const VREF: u32 = 1200;

/// ADC configuration
pub struct Adc {
    rb: ADC,
    /// The sample time to use for one-off conversions.
    pub sample_time: SampleTime,
    align: Align,
    clocks: Clocks,
    /// The value read from the Vref channel. This is used to scale readings appropriately.
    vref_value: u16,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(non_camel_case_types)]
/// ADC sampling time
///
/// Options for the sampling time, each is T + 0.5 ADC clock cycles.
pub enum SampleTime {
    /// 1.5 cycles sampling time
    Cycles1_5,
    /// 7.5 cycles sampling time
    Cycles7_5,
    /// 13.5 cycles sampling time
    Cycles13_5,
    /// 28.5 cycles sampling time
    Cycles28_5,
    /// 41.5 cycles sampling time
    Cycles41_5,
    /// 55.5 cycles sampling time
    Cycles55_5,
    /// 71.5 cycles sampling time
    Cycles71_5,
    /// 239.5 cycles sampling time
    Cycles239_5,
}

impl Default for SampleTime {
    /// Get the default sample time (currently 28.5 cycles)
    fn default() -> Self {
        SampleTime::Cycles28_5
    }
}

impl From<SampleTime> for SPT0_A {
    fn from(sample_time: SampleTime) -> Self {
        match sample_time {
            SampleTime::Cycles1_5 => Self::CYCLES1_5,
            SampleTime::Cycles7_5 => Self::CYCLES7_5,
            SampleTime::Cycles13_5 => Self::CYCLES13_5,
            SampleTime::Cycles28_5 => Self::CYCLES28_5,
            SampleTime::Cycles41_5 => Self::CYCLES41_5,
            SampleTime::Cycles55_5 => Self::CYCLES55_5,
            SampleTime::Cycles71_5 => Self::CYCLES71_5,
            SampleTime::Cycles239_5 => Self::CYCLES239_5,
        }
    }
}

impl From<SampleTime> for SPT10_A {
    fn from(sample_time: SampleTime) -> Self {
        match sample_time {
            SampleTime::Cycles1_5 => Self::CYCLES1_5,
            SampleTime::Cycles7_5 => Self::CYCLES7_5,
            SampleTime::Cycles13_5 => Self::CYCLES13_5,
            SampleTime::Cycles28_5 => Self::CYCLES28_5,
            SampleTime::Cycles41_5 => Self::CYCLES41_5,
            SampleTime::Cycles55_5 => Self::CYCLES55_5,
            SampleTime::Cycles71_5 => Self::CYCLES71_5,
            SampleTime::Cycles239_5 => Self::CYCLES239_5,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// ADC data register alignment
pub enum Align {
    /// Right alignment of output data
    Right,
    /// Left alignment of output data
    Left,
}

impl Default for Align {
    /// Default: right alignment
    fn default() -> Self {
        Align::Right
    }
}

impl From<Align> for DAL_A {
    fn from(alignment: Align) -> Self {
        match alignment {
            Align::Right => Self::RIGHT,
            Align::Left => Self::LEFT,
        }
    }
}

impl Adc {
    /// Initialises the ADC.
    ///
    /// Sets all configurable parameters to one-shot defaults, and performs a boot-time calibration.
    pub fn new(adc: ADC, apb2: &mut APB2, clocks: Clocks) -> Self {
        let mut s = Self {
            rb: adc,
            sample_time: SampleTime::default(),
            align: Align::default(),
            clocks,
            vref_value: 0, // This will be overwritten by `read_vref` below.
        };
        ADC::enable(apb2);
        s.power_down();
        ADC::reset(apb2);

        s.setup_oneshot();

        s.power_up();
        // Wait for the ADC to stabilise before starting calibration.
        delay(s.clocks.sysclk().0 / s.clocks.adcclk().0 * ADC_CALIBRATION_CYCLES);
        s.calibrate();

        // Read Vref so that it can be used to scale other readings.
        s.read_vref();

        s
    }

    /// Powers down the ADC, disables the ADC clock and releases the ADC peripheral.
    pub fn release(mut self, apb2: &mut APB2) -> ADC {
        self.power_down();
        ADC::disable(apb2);
        self.rb
    }

    fn power_up(&mut self) {
        self.rb.ctl1.modify(|_, w| w.adcon().enabled());
    }

    fn power_down(&mut self) {
        self.rb.ctl1.modify(|_, w| w.adcon().disabled());
    }

    fn calibrate(&mut self) {
        // Reset calibration.
        self.rb.ctl1.modify(|_, w| w.rstclb().start());
        while self.rb.ctl1.read().rstclb().is_not_complete() {}

        // Calibrate.
        self.rb.ctl1.modify(|_, w| w.clb().start());
        while self.rb.ctl1.read().clb().is_not_complete() {}
    }

    fn setup_oneshot(&mut self) {
        self.rb
            .ctl1
            .modify(|_, w| w.ctn().single().eterc().enabled().etsrc().swrcst());

        self.rb
            .ctl0
            .modify(|_, w| w.sm().disabled().disrc().enabled());

        // One channel in regular group
        self.rb.rsq0.modify(|_, w| w.rl().bits(0));
    }

    fn set_channel_sample_time(&mut self, channel: u8, sample_time: SampleTime) {
        match channel {
            0 => self
                .rb
                .sampt1
                .modify(|_, w| w.spt0().variant(sample_time.into())),
            1 => self
                .rb
                .sampt1
                .modify(|_, w| w.spt1().variant(sample_time.into())),
            2 => self
                .rb
                .sampt1
                .modify(|_, w| w.spt2().variant(sample_time.into())),
            3 => self
                .rb
                .sampt1
                .modify(|_, w| w.spt3().variant(sample_time.into())),
            4 => self
                .rb
                .sampt1
                .modify(|_, w| w.spt4().variant(sample_time.into())),
            5 => self
                .rb
                .sampt1
                .modify(|_, w| w.spt5().variant(sample_time.into())),
            6 => self
                .rb
                .sampt1
                .modify(|_, w| w.spt6().variant(sample_time.into())),
            7 => self
                .rb
                .sampt1
                .modify(|_, w| w.spt7().variant(sample_time.into())),
            8 => self
                .rb
                .sampt1
                .modify(|_, w| w.spt8().variant(sample_time.into())),
            9 => self
                .rb
                .sampt1
                .modify(|_, w| w.spt9().variant(sample_time.into())),

            10 => self
                .rb
                .sampt0
                .modify(|_, w| w.spt10().variant(sample_time.into())),
            11 => self
                .rb
                .sampt0
                .modify(|_, w| w.spt11().variant(sample_time.into())),
            12 => self
                .rb
                .sampt0
                .modify(|_, w| w.spt12().variant(sample_time.into())),
            13 => self
                .rb
                .sampt0
                .modify(|_, w| w.spt13().variant(sample_time.into())),
            14 => self
                .rb
                .sampt0
                .modify(|_, w| w.spt14().variant(sample_time.into())),
            15 => self
                .rb
                .sampt0
                .modify(|_, w| w.spt15().variant(sample_time.into())),
            16 => self
                .rb
                .sampt0
                .modify(|_, w| w.spt16().variant(sample_time.into())),
            17 => self
                .rb
                .sampt0
                .modify(|_, w| w.spt17().variant(sample_time.into())),
            18 => {
                #[cfg(any(feature = "gd32f130", feature = "gd32f150"))]
                self.set_channel_sample_time(0, sample_time);
                #[cfg(any(feature = "gd32f170", feature = "gd32f190"))]
                self.rb
                    .sampt0
                    .modify(|_, w| w.spt18().variant(sample_time.into()));
            }
            _ => unreachable!(),
        }
    }

    /// Set ADC sampling time for particular channel
    #[inline(always)]
    pub fn set_sample_time<C: Channel<ADC, ID = u8>>(&mut self, _pin: &C, sample_time: SampleTime) {
        self.set_channel_sample_time(C::channel(), sample_time);
    }

    /// Configures the ADC for a regular channel conversion sequence.
    pub fn with_regular_sequence(mut self, sequence: Sequence) -> SequenceAdc {
        self.set_regular_sequence_channels(sequence.channels());
        SequenceAdc {
            adc: self,
            sequence,
        }
    }

    /// Reads the internal reference voltage which is connected to channel 17 of the ADC.
    ///
    /// This both returns the value and stores it for converting other channel readings to voltages.
    /// Vref should always be 1.2 V.
    pub fn read_vref(&mut self) -> u16 {
        self.vref_value = self.read_aux(VRef::channel());
        self.vref_value
    }

    /// Reads the internal temperature sensor.
    ///
    /// It is recommended to set the sampling time to at least 17.1 µs before calling this. Returns
    /// the temperature in °C, assuming typical calibration values.
    pub fn read_temperature(&mut self) -> u16 {
        let vtemp_value = self.read_aux(VTemp::channel());
        self.calculate_temperature(vtemp_value)
    }

    /// Given an ADC reading, calculate the actual voltage based on the previous reading of Vref.
    pub fn calculate_voltage(&self, value: u16) -> u16 {
        (value as u32 * VREF / self.vref_value as u32) as u16
    }

    /// Calculates the temperature in °C from the given raw ADC reading, assuming typical
    /// calibration values.
    pub fn calculate_temperature(&self, vtemp_value: u16) -> u16 {
        let vtemp = self.calculate_voltage(vtemp_value);
        (VTEMP_25 - vtemp) * 10 / VTEMP_SLOPE + 25
    }

    /// Reads the backup battery voltage from channel 18 of the ADC.
    pub fn read_vbat(&mut self) -> u16 {
        let vbat_off = if self.rb.ctl1.read().vbaten().is_disabled() {
            self.rb.ctl1.modify(|_, w| w.vbaten().enabled());
            true
        } else {
            false
        };

        let value = self.convert(VBat::channel());

        if vbat_off {
            self.rb.ctl1.modify(|_, w| w.vbaten().disabled());
        }

        // Vbat/2 is connected to ADC channel 18, so we need to double it again.
        self.calculate_voltage(value) * 2
    }

    /// Reads the temperature sensor or Vref on channel 16 or 17.
    fn read_aux(&mut self, channel: u8) -> u16 {
        let tsv_off = if self.rb.ctl1.read().tsvren().is_disabled() {
            self.rb.ctl1.modify(|_, w| w.tsvren().enabled());
            true
        } else {
            false
        };

        let val = self.convert(channel);

        if tsv_off {
            self.rb.ctl1.modify(|_, w| w.tsvren().disabled());
        }

        val
    }

    fn set_regular_sequence_channels(&mut self, channels: &[u8]) {
        assert!(channels.iter().all(|channel| *channel <= 18));
        let len = channels.len();
        let bits = channels
            .iter()
            .take(6)
            .enumerate()
            .fold(0u32, |s, (i, c)| s | ((*c as u32) << (i * 5)));
        self.rb.rsq2.write(|w| unsafe { w.bits(bits) });
        if len > 6 {
            let bits = channels
                .iter()
                .skip(6)
                .take(6)
                .enumerate()
                .fold(0u32, |s, (i, c)| s | ((*c as u32) << (i * 5)));
            self.rb.rsq1.write(|w| unsafe { w.bits(bits) });
        }
        if len > 12 {
            let bits = channels
                .iter()
                .skip(12)
                .take(4)
                .enumerate()
                .fold(0u32, |s, (i, c)| s | ((*c as u32) << (i * 5)));
            self.rb.rsq0.write(|w| unsafe { w.bits(bits) });
        }
        self.rb.rsq0.modify(|_, w| w.rl().bits((len - 1) as u8));
    }

    /// Performs an ADC conversion for a single channel, with the default sample time.
    ///
    /// NOTE: Conversions can be started by writing a 1 to the ADCON bit in the `CTL1` while it is
    /// already 1, and no other bits are being written in the same operation. This means that the
    /// EOC bit *might* be set already when entering this function which can cause a read of stale
    /// values The check for `CTL.SWRCST` *should* fix it, but does not. Therefore, ensure you do
    /// not do any no-op modifications to `CTL1` just before calling this function.
    fn convert(&mut self, chan: u8) -> u16 {
        // Dummy read in case something accidentally triggered
        // a conversion by writing to CR2 without changing any
        // of the bits
        self.rb.rdata.read().rdata().bits();

        self.set_channel_sample_time(chan, self.sample_time);
        self.set_regular_sequence_channels(&[chan]);

        // Start conversion of regular sequence.
        self.rb
            .ctl1
            .modify(|_, w| w.swrcst().start().dal().variant(self.align.into()));
        while self.rb.ctl1.read().swrcst().is_not_started() {}

        // Wait for conversion results.
        while self.rb.stat.read().eoc().is_not_complete() {}

        self.rb.rdata.read().rdata().bits()
    }

    /// Configure the ADC to read from the given pin with DMA.
    pub fn with_dma<PIN>(mut self, pins: PIN, dma_ch: C0) -> AdcDma<PIN, Continuous>
    where
        PIN: Channel<ADC, ID = u8>,
    {
        self.rb.ctl0.modify(|_, w| w.disrc().disabled());
        self.rb
            .ctl1
            .modify(|_, w| w.dal().variant(self.align.into()));
        self.set_channel_sample_time(PIN::channel(), self.sample_time);
        self.set_regular_sequence_channels(&[PIN::channel()]);
        self.rb.ctl1.modify(|_, w| w.dma().enabled());

        let payload = AdcPayload {
            adc: self,
            pins,
            _mode: PhantomData,
        };
        RxDma {
            payload,
            channel: dma_ch,
        }
    }
}

impl<WORD, PIN> OneShot<ADC, WORD, PIN> for Adc
where
    WORD: From<u16>,
    PIN: Channel<ADC, ID = u8>,
{
    type Error = Infallible;

    fn read(&mut self, _pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
        let res = self.convert(PIN::channel());
        // TODO: Should this also be scaled based on Vref?
        Ok(res.into())
    }
}

/// ADC configured to convert a regular sequence of channels.
pub struct SequenceAdc {
    adc: Adc,
    sequence: Sequence,
}

impl SequenceAdc {
    /// Sets ADC continuous conversion
    ///
    /// When continuous conversion is enabled conversion does not stop at the last selected group
    /// channel but continues again from the first selected group channel.
    pub fn set_continuous_mode(&mut self, continuous: CTN_A) {
        self.adc.rb.ctl1.modify(|_, w| w.ctn().variant(continuous));
    }

    /// Sets ADC discontinuous mode
    ///
    /// It can be used to convert a short sequence of conversions (up to 8) which is a part of the
    /// regular sequence of conversions.
    pub fn set_discontinuous_mode(&mut self, channels_count: Option<u8>) {
        self.adc.rb.ctl0.modify(|_, w| match channels_count {
            Some(count) => w.disrc().enabled().disnum().bits(count),
            None => w.disrc().disabled(),
        });
    }

    /// Resets the ADC to one-shot mode, and releases the sequence that was previously configured.
    pub fn release(mut self) -> (Adc, Sequence) {
        // Reset configuration
        self.adc.rb.ctl0.modify(|_, w| w.disnum().bits(0));
        self.adc.setup_oneshot();
        (self.adc, self.sequence)
    }

    /// Configure the ADC to run with DMA. Before calling this make sure to set the
    pub fn with_scan_dma(self, dma_ch: C0) -> AdcDma<Sequence, Scan> {
        self.adc.rb.ctl1.modify(|_, w| {
            w.adcon()
                .disabled()
                .dma()
                .disabled()
                .ctn()
                .single()
                .dal()
                .variant(self.adc.align.into())
        });
        self.adc
            .rb
            .ctl0
            .modify(|_, w| w.sm().enabled().disrc().disabled());
        self.adc
            .rb
            .ctl1
            .modify(|_, w| w.dma().enabled().adcon().enabled());

        let payload = AdcPayload {
            adc: self.adc,
            pins: self.sequence,
            _mode: PhantomData,
        };
        RxDma {
            payload,
            channel: dma_ch,
        }
    }
}

/// A sequence of up to 16 channels for the ADC to convert.
#[derive(Debug, Default)]
pub struct Sequence {
    length: usize,
    channels: [u8; 16],
}

impl Sequence {
    /// Adds the given ADC pin to the list of channels.
    pub fn add_pin<PIN: Channel<ADC, ID = u8>>(&mut self, pin: PIN) -> Result<(), PIN> {
        if self.length >= self.channels.len() {
            return Err(pin);
        }
        self.channels[self.length] = PIN::channel();
        self.length += 1;
        Ok(())
    }

    /// Returns the number of channels
    pub fn len(&self) -> usize {
        self.length
    }

    fn channels(&self) -> &[u8] {
        &self.channels[0..self.length]
    }
}

/// Internal temperature sensor (ADC channel 16)
#[derive(Debug, Default)]
pub struct VTemp;

/// Internal voltage reference (ADC channel 17)
#[derive(Debug, Default)]
pub struct VRef;

/// Backup battery voltage / 2 (ADC channel 18)
#[derive(Debug, Default)]
pub struct VBat;

macro_rules! adc_pins {
    ($ADC:ident, $($pin:ty => $chan:expr),+ $(,)*) => {
        $(
            impl Channel<$ADC> for $pin {
                type ID = u8;

                fn channel() -> u8 { $chan }
            }
        )+
    };
}

adc_pins!(ADC,
    gpioa::PA0<Analog> => 0,
    gpioa::PA1<Analog> => 1,
    gpioa::PA2<Analog> => 2,
    gpioa::PA3<Analog> => 3,
    gpioa::PA4<Analog> => 4,
    gpioa::PA5<Analog> => 5,
    gpioa::PA6<Analog> => 6,
    gpioa::PA7<Analog> => 7,
    gpiob::PB0<Analog> => 8,
    gpiob::PB1<Analog> => 9,
    gpioc::PC0<Analog> => 10,
    gpioc::PC1<Analog> => 11,
    gpioc::PC2<Analog> => 12,
    gpioc::PC3<Analog> => 13,
    gpioc::PC4<Analog> => 14,
    gpioc::PC5<Analog> => 15,
    VTemp => 16,
    VRef => 17,
    VBat => 18,
);

pub struct AdcPayload<PINS, MODE> {
    adc: Adc,
    pins: PINS,
    _mode: PhantomData<MODE>,
}

pub type AdcDma<PINS, MODE> = RxDma<AdcPayload<PINS, MODE>, C0>;

impl<PINS, MODE> Receive for AdcDma<PINS, MODE> {
    type RxChannel = C0;
    type TransmittedWord = u16;
}

/// Continuous mode
pub struct Continuous;
/// Scan mode
pub struct Scan;

impl<PINS> TransferPayload for AdcDma<PINS, Continuous> {
    fn start(&mut self) {
        self.channel.start();
        self.payload.adc.rb.ctl1.modify(|_, w| w.ctn().continuous());
        // TODO: Is this a reliable way to trigger?
        self.payload.adc.rb.ctl1.modify(|_, w| w.adcon().enabled());
    }

    fn stop(&mut self) {
        self.channel.stop();
        self.payload.adc.rb.ctl1.modify(|_, w| w.ctn().single());
    }
}

impl TransferPayload for AdcDma<Sequence, Scan> {
    fn start(&mut self) {
        self.channel.start();
        self.payload.adc.rb.ctl1.modify(|_, w| w.adcon().enabled());
    }

    fn stop(&mut self) {
        self.channel.stop();
    }
}

impl<PINS> AdcDma<PINS, Continuous>
where
    Self: TransferPayload,
{
    pub fn split(mut self) -> (Adc, PINS, C0) {
        self.stop();

        let AdcDma { payload, channel } = self;
        payload.adc.rb.ctl1.modify(|_, w| w.dma().disabled());
        payload.adc.rb.ctl0.modify(|_, w| w.disrc().enabled());

        (payload.adc, payload.pins, channel)
    }
}

impl AdcDma<Sequence, Scan>
where
    Self: TransferPayload,
{
    pub fn split(mut self) -> (Adc, Sequence, C0) {
        self.stop();

        let AdcDma { payload, channel } = self;
        payload.adc.rb.ctl1.modify(|_, w| w.dma().disabled());
        payload
            .adc
            .rb
            .ctl0
            .modify(|_, w| w.disrc().enabled().sm().disabled());

        (payload.adc, payload.pins, channel)
    }
}

impl<B, PINS, MODE> CircReadDma<B, u16> for AdcDma<PINS, MODE>
where
    Self: TransferPayload,
    &'static mut [B; 2]: StaticWriteBuffer<Word = u16>,
    B: 'static,
{
    fn circ_read(mut self, mut buffer: &'static mut [B; 2]) -> CircBuffer<B, Self> {
        // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
        // until the end of the transfer.
        let (ptr, len) = unsafe { buffer.static_write_buffer() };
        self.channel
            .set_peripheral_address(unsafe { &(*ADC::ptr()).rdata as *const _ as u32 }, false);
        self.channel.set_memory_address(ptr as u32, true);
        self.channel.set_transfer_length(len);

        atomic::compiler_fence(Ordering::Release);

        self.channel.configure_from_peripheral(
            Priority::Medium,
            Width::Bits16,
            Width::Bits16,
            true,
        );

        self.start();

        CircBuffer::new(buffer, self)
    }
}

impl<B, PINS, MODE> ReadDma<B, u16> for AdcDma<PINS, MODE>
where
    Self: TransferPayload,
    B: StaticWriteBuffer<Word = u16>,
{
    fn read(mut self, mut buffer: B) -> Transfer<W, B, Self> {
        // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
        // until the end of the transfer.
        let (ptr, len) = unsafe { buffer.static_write_buffer() };
        self.channel
            .set_peripheral_address(unsafe { &(*ADC::ptr()).rdata as *const _ as u32 }, false);
        self.channel.set_memory_address(ptr as u32, true);
        self.channel.set_transfer_length(len);

        atomic::compiler_fence(Ordering::Release);
        self.channel.configure_from_peripheral(
            Priority::Medium,
            Width::Bits16,
            Width::Bits16,
            false,
        );
        self.start();

        Transfer::w(buffer, self)
    }
}
