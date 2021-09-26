# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [unreleased]

### Breaking changes

- Updated to `gd32f1` 0.5.0.

### New features

- Added initial support for GD32F150, GD32F170 and GD32F190 devices.

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

[unreleased]: https://github.com/qwandor/gd32f1x0-hal/compare/0.5.0...HEAD
[0.2.0]: https://github.com/qwandor/gd32f1x0-hal/compare/0.1.0...0.2.0
[0.2.1]: https://github.com/qwandor/gd32f1x0-hal/compare/0.2.0...0.2.1
[0.3.0]: https://github.com/qwandor/gd32f1x0-hal/compare/0.2.1...0.3.0
[0.4.0]: https://github.com/qwandor/gd32f1x0-hal/compare/0.3.0...0.4.0
[0.5.0]: https://github.com/qwandor/gd32f1x0-hal/compare/0.4.0...0.5.0
