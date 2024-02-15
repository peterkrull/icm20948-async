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

## Example use

```rust
use icm20948_async::{Icm20948, GyrUnit, GyrDlp, AccRange};

let imu_result = Icm20948::new_i2c(i2c, delay)
        .gyr_unit(GyrUnit::Rps)   // Set gyroscope to output rad/s
        .gyr_dlp(GyrDlp::Hz196)   // Set gyroscope low-pass filter
        .acc_range(AccRange::Gs8) // Set accelerometer measurement range
        .set_address(0x69)        // Set address (0x68 or 0x69)
        .initialize_9dof().await; // Initialize with magnetometer
// where
//     i2c: impl embedded_hal_async::i2c::I2c
//     delay: impl embedded_hal_async::delay::DelayNs

let Ok(mut imu) = imu_result else { panic!("Failed to initialize IMU") };

loop {
    // Read data from IMU, loop back in case of failure
    let Ok(measurement) = imu.read_9dof().await else { continue };

    // We now have:
    // measurement.acc: nalgebra::Vector3<f32>
    // measurement.gyr: nalgebra::Vector3<f32>
    // measurement.mag: nalgebra::Vector3<f32>
}

```
