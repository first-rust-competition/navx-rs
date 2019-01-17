# navx-rs [![CircleCI][status-badge]][circle-ci-link]
Rust support for frc NavX build on top of [wpilib](https://github.com/Lytigas/first-rust-competition).

Currently, only the SPI interface and get_yaw are implemented. The framework is in place for all the other register based protocols, though. Note that this has not been rigorously tested. Any pull requests or bug reports are highly appreciated!

## Features
The `nightly` feature enables certain optimizations provided by the `nightly`
feature of [`parking_lot`]; in particular, it allows some initialization
to be done at compile time.
## License

The contents of this repository are distributed under the terms of both the
MIT license and the Apache License (Version 2.0). By contributing, you agree
to license your contribution under these terms.

See [LICENSE-APACHE](LICENSE-APACHE), [LICENSE-MIT](LICENSE-MIT), for details.

[`parking_lot`]: https://github.com/Amanieu/parking_lot
[status-badge]:
https://circleci.com/gh/Eaglestrike/navx-rs.svg?style=shield
[circle-ci-link]:
https://circleci.com/gh/Eaglestrike/navx-rs
