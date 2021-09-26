// Copyright 2021 The gd32f1x0-hal authors.
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! # HAL for the GD32F1x0 family of microcontrollers
//!
//! This is an implementation of the [`embedded-hal`] traits for the GD32F1x0 family of
//! microcontrollers.
//!
//! [`embedded-hal`]: https://crates.io/crates/embedded-hal
//!
//! # Usage
//!
//! ## Building an application (binary crate)
//!
//! A detailed usage guide can be found in the [README]
//!
//! ## Variants
//!
//! This crate supports multiple microcontrollers in the GD32F1x0 family. Which specific
//! microcontroller you want to build for has to be specified with a feature, for example
//! `gd32f130x8`.
//!
//! If no microcontroller is specified, the crate will not compile.
//!
//! The currently supported variants are
//!
//! - `gd32f130x4` (e.g. GD32F130F4, GD32F130G4, ...)
//! - `gd32f130x6` (e.g. GD32F130F6, GD32F130G6, ...)
//! - `gd32f130x8` (e.g. GD32F130F8, GD32F130G8, ...)
//! - `gd32f150x4` (e.g. GD32F150G4, GD32F150K4, ...)
//! - `gd32f150x6` (e.g. GD32F150G6, GD32F150K6, ...)
//! - `gd32f150x8` (e.g. GD32F150G8, GD32F150K8, ...)
//! - `gd32f170x4` (e.g. GD32F170T4, GD32F170C4, ...)
//! - `gd32f170x6` (e.g. GD32F170T6, GD32F170C6, ...)
//! - `gd32f170x8` (e.g. GD32F170T8, GD32F170C8, ...)
//! - `gd32f190x4` (e.g. GD32F190T4, GD32F190C4, ...)
//! - `gd32f190x6` (e.g. GD32F190T6, GD32F190C6, ...)
//! - `gd32f190x8` (e.g. GD32F190T8, GD32F190C8, ...)
//!
//! ## Commonly used setup
//! Almost all peripherals require references to some registers in `RCU`. The following
//! code shows how to set up those registers
//!
//! ```rust
//! // Get access to the device specific peripherals from the peripheral access crate
//! let dp = pac::Peripherals::take().unwrap();
//!
//! // Take ownership over the raw RCU and FMC devices and convert tem into the corresponding HAL
//! //  structs.
//! let mut rcu = dp.RCU.constrain();
//! let mut flash = p.FMC.constrain();
//!
//! // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
//! // `clocks`
//! let clocks = rcu.cfgr.freeze(&mut flash.ws);
//! ```
//!
//! ## Usage examples
//!
//! See the [examples] folder.
//!
//! Most of the examples require the following additional dependencies
//! ```toml
//! [dependencies]
//! embedded-hal = "0.2.3"
//! nb = "0.1.2"
//! cortex-m = "0.6.2"
//! cortex-m-rt = "0.6.11"
//! # Panic behaviour, see https://crates.io/keywords/panic-impl for alternatives
//! panic-halt = "0.2.0"
//! ```
//!
//! [examples]: https://github.com/qwandor/gd32f1x0-hal/tree/main/examples
//! [README]: https://github.com/qwandor/gd32f1x0-hal

#![no_std]
#![deny(broken_intra_doc_links)]

// If no target specified, print error message.
#[cfg(not(any(
    feature = "gd32f130x4",
    feature = "gd32f130x6",
    feature = "gd32f130x8",
    feature = "gd32f150x4",
    feature = "gd32f150x6",
    feature = "gd32f150x8",
    feature = "gd32f170x4",
    feature = "gd32f170x6",
    feature = "gd32f170x8",
    feature = "gd32f190x4",
    feature = "gd32f190x6",
    feature = "gd32f190x8",
)))]
compile_error!("Target not found. A `--features <target-name>` is required.");

// If any two or more targets are specified, print error message.
#[cfg(any(
    all(feature = "gd32f130", feature = "gd32f150"),
    all(feature = "gd32f130", feature = "gd32f170"),
    all(feature = "gd32f130", feature = "gd32f190"),
    all(feature = "gd32f150", feature = "gd32f170"),
    all(feature = "gd32f150", feature = "gd32f190"),
    all(feature = "gd32f170", feature = "gd32f190"),
    all(feature = "gd32f130x4", feature = "gd32f130x6"),
    all(feature = "gd32f130x4", feature = "gd32f130x8"),
    all(feature = "gd32f130x6", feature = "gd32f130x8"),
    all(feature = "gd32f150x4", feature = "gd32f150x6"),
    all(feature = "gd32f150x4", feature = "gd32f150x8"),
    all(feature = "gd32f150x6", feature = "gd32f150x8"),
    all(feature = "gd32f170x4", feature = "gd32f170x6"),
    all(feature = "gd32f170x4", feature = "gd32f170x8"),
    all(feature = "gd32f170x6", feature = "gd32f170x8"),
    all(feature = "gd32f190x4", feature = "gd32f190x6"),
    all(feature = "gd32f190x4", feature = "gd32f190x8"),
    all(feature = "gd32f190x6", feature = "gd32f190x8"),
))]
compile_error!(
    "Multiple targets specified. Only a single `--features <target-name>` can be specified."
);

#[cfg(feature = "gd32f130")]
pub use gd32f1::gd32f130 as pac;
#[cfg(feature = "gd32f150")]
pub use gd32f1::gd32f150 as pac;
#[cfg(feature = "gd32f170")]
pub use gd32f1::gd32f170 as pac;
#[cfg(feature = "gd32f190")]
pub use gd32f1::gd32f190 as pac;

#[cfg(feature = "device-selected")]
pub mod adc;
/*#[cfg(feature = "device-selected")]
pub mod backup_domain;
#[cfg(all(feature = "device-selected", feature = "has-can"))]
pub mod can;*/
#[cfg(feature = "device-selected")]
pub mod crc;
#[cfg(feature = "device-selected")]
pub mod delay;
#[cfg(feature = "device-selected")]
pub mod dma;
#[cfg(feature = "device-selected")]
pub mod flash;
#[cfg(feature = "device-selected")]
pub mod gpio;
/*#[cfg(feature = "device-selected")]
pub mod i2c;*/
#[cfg(feature = "device-selected")]
pub mod prelude;
#[cfg(feature = "device-selected")]
pub mod pwm;
/*#[cfg(feature = "device-selected")]
pub mod pwm_input;
#[cfg(feature = "device-selected")]
pub mod qei;*/
#[cfg(feature = "device-selected")]
pub mod rcu;
/*#[cfg(feature = "device-selected")]
pub mod rtc;*/
#[cfg(feature = "device-selected")]
pub mod serial;
/*#[cfg(feature = "device-selected")]
pub mod spi;*/
#[cfg(feature = "device-selected")]
pub mod time;
#[cfg(feature = "device-selected")]
pub mod timer;
/*#[cfg(all(
    feature = "stm32-usbd",
    any(feature = "stm32f102", feature = "stm32f103")
))]
pub mod usb;*/
#[cfg(feature = "device-selected")]
pub mod watchdog;
