// Copyright 2021 The gd32f1x0-hal authors.
//
// SPDX-License-Identifier: MIT OR Apache-2.0

/*
pub use crate::crc::CrcExt as _gd32_hal_crc_CrcExt;
pub use crate::flash::FlashExt as _gd32_hal_flash_FlashExt;*/
pub use crate::dma::CircReadDma as _gd32_hal_dma_CircReadDma;
pub use crate::dma::DmaExt as _gd32_hal_dma_DmaExt;
pub use crate::dma::ReadDma as _gd32_hal_dma_ReadDma;
pub use crate::dma::WriteDma as _gd32_hal_dma_WriteDma;
pub use crate::gpio::GpioExt as _gd32_hal_gpio_GpioExt;
pub use crate::rcu::RcuExt as _gd32_hal_rcu_RcuExt;
pub use crate::time::U32Ext as _gd32_hal_time_U32Ext;
pub use embedded_hal::adc::OneShot as _embedded_hal_adc_OneShot;
pub use embedded_hal::digital::v2::InputPin as _embedded_hal_digital_InputPin;
pub use embedded_hal::digital::v2::OutputPin as _embedded_hal_digital_OutputPin;
pub use embedded_hal::digital::v2::StatefulOutputPin as _embedded_hal_digital_StatefulOutputPin;
pub use embedded_hal::digital::v2::ToggleableOutputPin as _embedded_hal_digital_ToggleableOutputPin;
pub use embedded_hal::prelude::*;
pub use embedded_hal::watchdog::Watchdog as _embedded_watchdog_Watchdog;
pub use embedded_hal::watchdog::WatchdogEnable as _embedded_watchdog_WatchdogEnable;
