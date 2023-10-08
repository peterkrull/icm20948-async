# Async Rust driver for the ICM20948

This driver crate for the ICM20948 uses the `embedded-hal-async` traits to achieve completely non-blocking access to the ICM20948. The crate uses generics to automatically expose the relevant methods, when various hardware features are configured, for example when the magnetometer is enabled.

The current feature set is basic, but allows for reading the main sensors and writing the most important values. Below is the feature list, with supported features checked off. The rest are on the to-do list. 

- [x] Reading from accelerometer
- [x] Reading from gyroscope
- [x] Reading from magnetometer
- [x] Reading from thermometer
- [x] I2C support (async)
- [x] SPI support (async)
- [ ] I2C support (blocking)
- [ ] SPI support (blocking)
- [x] Setting DLP, range and unit
- [x] Setting sample rate divider
- [x] Setting offsets (acc, gyro)
- [ ] Support for FIFO
- [ ] Support for DMP
- [ ] Power management
- [ ] Run self-tests
- [ ] Interrupts
- [x] Release I2C/SPI object when not used
- [x] Use embedded-hal traits for delays