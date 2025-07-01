// Copyright 2021 The gd32f1x0-hal authors.
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Flash memory

use crate::pac::{Fmc, fmc};

pub const FLASH_START: u32 = 0x0800_0000;
pub const FLASH_END: u32 = 0x080F_FFFF;

const _RDPRT_KEY: u16 = 0x00A5;
const KEY1: u32 = 0x45670123;
const KEY2: u32 = 0xCDEF89AB;

pub const SZ_1K: u16 = 1024;

pub type Result<T> = core::result::Result<T, Error>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd, thiserror::Error)]
pub enum Error {
    #[error("Address larger than flash")]
    AddressLargerThanFlash,
    #[error("Address misaligned")]
    AddressMisaligned,
    #[error("Length is not a multiple of 2")]
    LengthNotMultiple2,
    #[error("Length is too long")]
    LengthTooLong,
    #[error("Erase error")]
    EraseError,
    #[error("Programming error")]
    ProgrammingError,
    #[error("Write error")]
    WriteError,
    #[error("Verification error")]
    VerifyError,
    #[error("Unlock error")]
    UnlockError,
    #[error("Lock error")]
    LockError,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd)]
pub enum SectorSize {
    Sz1K = 1,
    Sz2K = 2,
    Sz4K = 4,
}
impl SectorSize {
    const fn kbytes(self) -> u16 {
        SZ_1K * self as u16
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd)]
pub enum FlashSize {
    Sz16K = 16,
    Sz32K = 32,
    Sz64K = 64,
    Sz128K = 128,
    Sz256K = 256,
    Sz384K = 384,
    Sz512K = 512,
    Sz768K = 768,
    Sz1M = 1024,
}

impl FlashSize {
    const fn kbytes(self) -> u32 {
        SZ_1K as u32 * self as u32
    }
}

pub struct FlashWriter<'a> {
    fmc: &'a mut Parts,
    sector_sz: SectorSize,
    flash_sz: FlashSize,
    verify: bool,
}

impl<'a> FlashWriter<'a> {
    fn unlock(&mut self) -> Result<()> {
        // Wait for any ongoing operations
        while self.fmc.stat.stat().read().busy().is_active() {}

        // NOTE(unsafe) write Keys to the key register. This is safe because the
        // only side effect of these writes is to unlock the fmc control
        // register, which is the intent of this function. Do not rearrange the
        // order of these writes or the control register will be permanently
        // locked out until reset.
        self.fmc.key.keyr().write(|w| w.key().bits(KEY1));
        self.fmc.key.keyr().write(|w| w.key().bits(KEY2));

        // Verify success
        match self.fmc.ctl.ctl().read().lk().is_unlocked() {
            true => Ok(()),
            false => Err(Error::UnlockError),
        }
    }

    fn lock(&mut self) -> Result<()> {
        // Wait for ongoing flash operations
        while self.fmc.stat.stat().read().busy().is_active() {}

        // Set lock bit
        self.fmc.ctl.ctl().modify(|_, w| w.lk().lock());

        // Verify success
        match self.fmc.ctl.ctl().read().lk().is_locked() {
            true => Ok(()),
            false => Err(Error::LockError),
        }
    }

    fn valid_address(&self, offset: u32) -> Result<()> {
        if FLASH_START + offset > FLASH_END {
            Err(Error::AddressLargerThanFlash)
        } else if offset & 0x1 != 0 {
            Err(Error::AddressMisaligned)
        } else {
            Ok(())
        }
    }

    fn valid_length(&self, offset: u32, length: usize) -> Result<()> {
        if offset + length as u32 > self.flash_sz.kbytes() {
            Err(Error::LengthTooLong)
        } else if length & 0x1 != 0 {
            Err(Error::LengthNotMultiple2)
        } else {
            Ok(())
        }
    }

    /// Erase sector which contains `start_offset`
    pub fn page_erase(&mut self, start_offset: u32) -> Result<()> {
        self.valid_address(start_offset)?;

        // Unlock Flash
        self.unlock()?;

        // Set Page Erase
        self.fmc.ctl.ctl().modify(|_, w| w.per().page_erase());

        // Write address bits
        // NOTE(unsafe) This sets the page address in the Address Register.
        // The side-effect of this write is that the page will be erased when we
        // set the STRT bit in the CTL below. The address is validated by the
        // call to self.valid_address() above.
        self.fmc
            .addr
            .addr()
            .write(|w| w.addr().bits(FLASH_START + start_offset));

        // Start Operation
        self.fmc.ctl.ctl().modify(|_, w| w.start().start());

        // Wait for operation to finish
        while self.fmc.stat.stat().read().busy().is_active() {}

        // Check for errors
        let stat = self.fmc.stat.stat().read();

        // Remove Page Erase Operation bit
        self.fmc.ctl.ctl().modify(|_, w| w.per().clear_bit());

        // Re-lock flash
        self.lock()?;

        if stat.wperr().is_error() {
            self.fmc.stat.stat().modify(|_, w| w.wperr().clear());
            Err(Error::EraseError)
        } else {
            if self.verify {
                // By subtracting 1 from the sector size and masking with
                // start_offset, we make 'start' point to the beginning of the
                // page. We do this because the entire page should have been
                // erased, regardless of where in the page the given
                // 'start_offset' was.
                let size = self.sector_sz.kbytes() as u32;
                let start = start_offset & !(size - 1);
                for idx in start..start + size {
                    let write_address = (FLASH_START + idx) as *const u16;
                    let verify: u16 = unsafe { core::ptr::read_volatile(write_address) };
                    if verify != 0xFFFF {
                        return Err(Error::VerifyError);
                    }
                }
            }

            Ok(())
        }
    }

    /// Erase the Flash Sectors from `FLASH_START + start_offset` to `length`
    pub fn erase(&mut self, start_offset: u32, length: usize) -> Result<()> {
        self.valid_length(start_offset, length)?;

        // Erase every sector touched by start_offset + length
        for offset in
            (start_offset..start_offset + length as u32).step_by(self.sector_sz.kbytes() as usize)
        {
            self.page_erase(offset)?;
        }

        // Report Success
        Ok(())
    }

    /// Retrieve a slice of data from `FLASH_START + offset`
    pub fn read(&self, offset: u32, length: usize) -> Result<&[u8]> {
        self.valid_address(offset)?;

        if offset + length as u32 > self.flash_sz.kbytes() {
            return Err(Error::LengthTooLong);
        }

        let address = (FLASH_START + offset) as *const _;

        Ok(
            // NOTE(unsafe) read with no side effects. The data returned will
            // remain valid for its lifetime because we take an immutable
            // reference to this FlashWriter, and any operation that would
            // invalidate the data returned would first require taking a mutable
            // reference to this FlashWriter.
            unsafe { core::slice::from_raw_parts(address, length) },
        )
    }

    /// Write data to `FLASH_START + offset`
    pub fn write(&mut self, offset: u32, data: &[u8]) -> Result<()> {
        self.valid_length(offset, data.len())?;

        // Unlock Flash
        self.unlock()?;

        for idx in (0..data.len()).step_by(2) {
            self.valid_address(offset + idx as u32)?;

            let write_address = (FLASH_START + offset + idx as u32) as *mut u16;

            // Set Page Programming to 1
            self.fmc.ctl.ctl().modify(|_, w| w.pg().program());

            while self.fmc.stat.stat().read().busy().is_active() {}

            // Flash is written 16 bits at a time, so combine two bytes to get a
            // half-word
            let hword: u16 = (data[idx] as u16) | (data[idx + 1] as u16) << 8;

            // NOTE(unsafe) Write to FLASH area with no side effects
            unsafe { core::ptr::write_volatile(write_address, hword) };

            // Wait for write
            while self.fmc.stat.stat().read().busy().is_active() {}

            // Set Page Programming to 0
            self.fmc.ctl.ctl().modify(|_, w| w.pg().clear_bit());

            // Check for errors
            if self.fmc.stat.stat().read().pgerr().is_error() {
                self.fmc.stat.stat().modify(|_, w| w.pgerr().clear_bit());

                self.lock()?;
                return Err(Error::ProgrammingError);
            } else if self.fmc.stat.stat().read().wperr().is_error() {
                self.fmc.stat.stat().modify(|_, w| w.wperr().clear());

                self.lock()?;
                return Err(Error::WriteError);
            } else if self.verify {
                // Verify written WORD
                // NOTE(unsafe) read with no side effects within FLASH area
                let verify: u16 = unsafe { core::ptr::read_volatile(write_address) };
                if verify != hword {
                    self.lock()?;
                    return Err(Error::VerifyError);
                }
            }
        }

        // Lock Flash and report success
        self.lock()?;
        Ok(())
    }

    /// Enable/disable verifying that each erase or write operation completed
    /// successfuly.
    ///
    /// When enabled, after each erase operation every address is read to make
    /// sure it contains the erase value of 0xFFFF. After each write operation,
    /// every address written is read and compared to the value that should have
    /// been written. If any address does not contain the expected value, the
    /// function will return Err.
    /// When disabled, no verification is performed, erase/write operations are
    /// assumed to have succeeded.
    pub fn change_verification(&mut self, verify: bool) {
        self.verify = verify;
    }
}

/// Extension trait to constrain the FMC peripheral
pub trait FlashExt {
    /// Constrains the FMC peripheral to play nicely with the other abstractions
    fn constrain(self) -> Parts;
}

impl FlashExt for Fmc {
    fn constrain(self) -> Parts {
        Parts {
            ws: WS { _0: () },
            addr: ADDR { _0: () },
            ctl: CTL { _0: () },
            key: KEY { _0: () },
            _obstat: OBSTAT { _0: () },
            _obkey: OBKEY { _0: () },
            stat: STAT { _0: () },
            _wp: WP { _0: () },
        }
    }
}

/// Constrained FMC peripheral
pub struct Parts {
    /// Opaque WS register
    pub ws: WS,

    /// Opaque ADDR register
    pub(crate) addr: ADDR,

    /// Opaque CTL register
    pub(crate) ctl: CTL,

    /// Opaque KEY register
    pub(crate) key: KEY,

    /// Opaque OBSTAT register
    pub(crate) _obstat: OBSTAT,

    /// Opaque OPTKEYR register
    pub(crate) _obkey: OBKEY,

    /// Opaque STAT register
    pub(crate) stat: STAT,

    /// Opaque WP register
    pub(crate) _wp: WP,
}
impl Parts {
    pub fn writer(&mut self, sector_sz: SectorSize, flash_sz: FlashSize) -> FlashWriter {
        FlashWriter {
            fmc: self,
            sector_sz,
            flash_sz,
            verify: true,
        }
    }
}

/// Opaque WS register
pub struct WS {
    _0: (),
}

#[allow(dead_code)]
impl WS {
    pub(crate) fn ws(&mut self) -> &fmc::Ws {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*Fmc::ptr()).ws() }
    }
}

/// Opaque ADDR register
pub struct ADDR {
    _0: (),
}

impl ADDR {
    pub(crate) fn addr(&mut self) -> &fmc::Addr {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*Fmc::ptr()).addr() }
    }
}

/// Opaque CTL register
pub struct CTL {
    _0: (),
}

impl CTL {
    pub(crate) fn ctl(&mut self) -> &fmc::Ctl {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*Fmc::ptr()).ctl() }
    }
}

/// Opaque KEY register
pub struct KEY {
    _0: (),
}

impl KEY {
    pub(crate) fn keyr(&mut self) -> &fmc::Key {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*Fmc::ptr()).key() }
    }
}

/// Opaque OBSTAT register
pub struct OBSTAT {
    _0: (),
}

#[allow(dead_code)]
impl OBSTAT {
    pub(crate) fn obstat(&mut self) -> &fmc::Obstat {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*Fmc::ptr()).obstat() }
    }
}

/// Opaque OBKEY register
pub struct OBKEY {
    _0: (),
}

#[allow(dead_code)]
impl OBKEY {
    pub(crate) fn optkeyr(&mut self) -> &fmc::Obkey {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*Fmc::ptr()).obkey() }
    }
}

/// Opaque STAT register
pub struct STAT {
    _0: (),
}

impl STAT {
    pub(crate) fn stat(&mut self) -> &fmc::Stat {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*Fmc::ptr()).stat() }
    }
}

/// Opaque WP register
pub struct WP {
    _0: (),
}

#[allow(dead_code)]
impl WP {
    pub(crate) fn wp(&mut self) -> &fmc::Wp {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*Fmc::ptr()).wp() }
    }
}
