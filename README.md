# Async Rust driver for the ICM20948

This driver crate for the ICM20948 uses the `embedded-hal-async` traits and the `embassy_time` crate to achieve non-blocking access to the ICM20948. The crate uses generics to automatically expose the relevant methods, when various hardware features are configured, for example when the magnetometer is enabled.

The current feature set is basic, but allows for reading the most important sensors. Below is the feature list, with supported features checked off. The rest are on the to-do list. 

- [x] Reading from accelerometer
- [x] Reading from gyroscope
- [x] Reading from magnetometer
- [ ] Reading from thermometer
- [x] I2C support (async)
- [ ] SPI support (async)
- [ ] I2C support (blocking)
- [ ] SPI support (blocking)
- [ ] Setting I2C config
- [ ] Setting SPI config
- [x] Setting DLP, range and unit
- [ ] Setting sample rate divider
- [ ] Setting offsets (acc, gyro)
- [ ] Support for FIFO
- [ ] Support for DMP
- [ ] Power management
- [ ] Run self-tests
- [ ] Interrupts

Other things I want to work on:
- [ ] Use embedded-hal traits for delays
- [ ] Built-in calibration sequences
- [ ] Release I2C/SPI object when not used
