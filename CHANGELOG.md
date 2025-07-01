# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [0.10.2]

### New features

- All error types now implement `core::error::Error`, `Display` and other useful traits.

## [0.10.1]

### Bugfixes

- Fixed integer overflow issue in `DelayNs` for `Delay`.

## [0.10.0]

### Breaking changes

- Updated to `gd32f1` 0.9.1.

### Other changes

- Removed unused dev-dependencies.

## [0.9.1]

### Other changes

- Removed unused `bxcan` dependency.
- Removed `cast` dependency.

## [0.9.0]

### Breaking changes

- Moved `embedded-hal` 0.2 trait implementations behind the `embedded-hal-02` feature flag,
  to make the dependency optional.

## [0.8.0]

### Breaking changes

- Removed `embedded-hal` 0.2 traits from prelude.
- Changed error type of `embedded-hal` 0.2 trait implementations for `BlockingI2c`.

### New features

- Implemented `embedded-hal` 1.0 traits where relevant.

## [0.7.1]

### Other changes

- Fixed link to documentation.

## [0.7.0]

### Breaking changes

- Updated to `bxcan` 0.7.0.
- Updated to `embedded-dma` 0.2.0.
- Updated to `gd32f1` 0.6.0.

### Other changes

- Updated various dependencies.

## [0.6.0]

### Breaking changes

- Updated to `gd32f1` 0.5.0.

### New features

- Added initial support for GD32F150, GD32F170 and GD32F190 devices.
- Added support for I2C.

## [0.5.0]

### Breaking changes

- Updated to `gd32f1` 0.4.0, which fixes a bug which prevented all timers apart from TIMER0 from
  working properly.

## [0.4.0]

### Breaking changes

- Changed `Pwm::automatic_output_disable` to `output_disable`, to actually disable the output.

## [0.3.0]

### Breaking changes

- Updated to `gd32f1` 0.3.0.
- Changed `CountDownTimer::clear_update_interrupt_flag()` to `CountDownTimer::clear_interrupt_flag(Event::Update)`.

### New features

- Added support for complementary channels and other related PWM features.
- Added `CountDownTimer::is_pending`.

## [0.2.1]

### Bugfixes

- Fixed PWM mode, which was incorrectly changed in 0.2.0 from mode 0 to mode 1. It is now mode 0 as
  it should be.
- Fixed bug with `Pwm::set_period` not working reliably.

## [0.2.0]

### Breaking changes

- Updated to `gd32f1` 0.2.1.
- Renamed `CrcExt::new` to `CrcExt::constrain` for consistency.

## [0.1.0]

First release.

[unreleased]: https://github.com/gd32-rust/gd32f1x0-hal/compare/0.10.2...HEAD
[0.2.0]: https://github.com/gd32-rust/gd32f1x0-hal/compare/0.1.0...0.2.0
[0.2.1]: https://github.com/gd32-rust/gd32f1x0-hal/compare/0.2.0...0.2.1
[0.3.0]: https://github.com/gd32-rust/gd32f1x0-hal/compare/0.2.1...0.3.0
[0.4.0]: https://github.com/gd32-rust/gd32f1x0-hal/compare/0.3.0...0.4.0
[0.5.0]: https://github.com/gd32-rust/gd32f1x0-hal/compare/0.4.0...0.5.0
[0.6.0]: https://github.com/gd32-rust/gd32f1x0-hal/compare/0.5.0...0.6.0
[0.7.0]: https://github.com/gd32-rust/gd32f1x0-hal/compare/0.6.0...0.7.0
[0.7.1]: https://github.com/gd32-rust/gd32f1x0-hal/compare/0.7.0...0.7.1
[0.8.0]: https://github.com/gd32-rust/gd32f1x0-hal/compare/0.7.1...0.8.0
[0.9.0]: https://github.com/gd32-rust/gd32f1x0-hal/compare/0.8.0...0.9.0
[0.9.1]: https://github.com/gd32-rust/gd32f1x0-hal/compare/0.9.0...0.9.1
[0.10.0]: https://github.com/gd32-rust/gd32f1x0-hal/compare/0.9.1...0.10.0
[0.10.1]: https://github.com/gd32-rust/gd32f1x0-hal/compare/0.10.0...0.10.1
[0.10.1]: https://github.com/gd32-rust/gd32f1x0-hal/compare/0.10.1...0.10.2
