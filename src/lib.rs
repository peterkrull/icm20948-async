#![no_std]

use core::{future::Future, marker::PhantomData};
use embedded_hal_async::{
    delay::DelayNs,
    i2c::{self, I2c},
    spi::{self},
};

mod reg;
use reg::*;

mod cfg;
pub use cfg::*;

const MAGNET_ADDR: u8 = 0x0C;
const IMU_WHOAMI: u8 = 0xEA;
const MAG_WHOAMI: u8 = 0x09;

#[derive(Debug, Clone, PartialEq)]
pub enum SetupError<E> {
    /// An error occured with the I2C/SPI connection during setup
    Bus(E),
    /// An incorrect 'Who Am I' value was returned from the imu, expected 0xEA (233)
    ImuWhoAmI(u8),
    /// An incorrect 'Who Am I' value was returned from the mag, expected 0x09 (9)
    MagWhoAmI(u8),
}

impl<E> From<E> for SetupError<E> {
    fn from(error: E) -> Self {
        SetupError::Bus(error)
    }
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
/// Container for accelerometer and gyroscope measurements
pub struct Data6Dof<T> {
    pub acc: [T; 3],
    pub gyr: [T; 3],
    pub tmp: T,
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
/// Container for accelerometer, gyroscope and magnetometer measurements
pub struct Data9Dof<T> {
    pub acc: [T; 3],
    pub gyr: [T; 3],
    pub mag: [T; 3],
    pub tmp: T,
}

// Compile-time MAG states
pub struct MagEnabled;
pub struct MagDisabled;

// Type to hold bus information for I2c
pub struct I2cDevice<I2C> {
    bus_inner: I2C,
    address: I2cAddress,
}

// Trait to allow for generic behavior across I2c or Spi usage
pub trait RegisterDevice {
    type Error: Into<SetupError<Self::Error>>;
    fn read_registers(
        &mut self,
        reg_addr: u8,
        read: &mut [u8],
    ) -> impl Future<Output = Result<(), Self::Error>>;
    fn write_registers(
        &mut self,
        reg_addr: u8,
        write: &[u8],
    ) -> impl Future<Output = Result<(), Self::Error>>;
}

// Implementation of register device trait for I2c
impl<I2C: I2c> RegisterDevice for I2cDevice<I2C> {
    type Error = I2C::Error;
    async fn read_registers(&mut self, reg_addr: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        let addr = self.address.get();
        self.bus_inner.write_read(addr, &[reg_addr], read).await
    }

    async fn write_registers(&mut self, reg_addr: u8, write: &[u8]) -> Result<(), Self::Error> {
        let addr = self.address.get();
        self.bus_inner
            .transaction(
                addr,
                &mut [
                    i2c::Operation::Write(&[reg_addr]),
                    i2c::Operation::Write(write),
                ],
            )
            .await
    }
}

mod spi_helpers {
    const SPI_READ_NWRITE: u8 = 0x80;
    pub(super) const fn write_addr(addr: u8) -> [u8; 1] {
        [addr]
    }
    // Read operation demands to firstly write a register address with MSB set
    pub(super) const fn read_addr(addr: u8) -> [u8; 1] {
        [addr | SPI_READ_NWRITE]
    }
}

pub struct SpiDevice<SPI: spi::SpiDevice> {
    inner: SPI,
}

// Implementation of register device trait for Spi
impl<SPI: spi::SpiDevice> RegisterDevice for SpiDevice<SPI> {
    type Error = SPI::Error;
    async fn read_registers(&mut self, reg_addr: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.inner
            .transaction(&mut [
                spi::Operation::Write(&spi_helpers::read_addr(reg_addr)),
                spi::Operation::Read(read),
            ])
            .await
    }

    async fn write_registers(&mut self, reg_addr: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.inner
            .transaction(&mut [
                spi::Operation::Write(&spi_helpers::write_addr(reg_addr)),
                spi::Operation::Write(write),
            ])
            .await
    }
}

/// Configure and initialize a new instance of [`Icm20948`]. Use either of the
/// [`Icm20948::new_i2c`] or [`Icm20948::new_spi`] methods to begin.
///
/// # Example
///
/// ```rust
/// let mut imu = IcmBuilder::new_i2c(i2c, delay)
///     .gyr_unit(GyrUnit::Rps)
///     .gyr_dlp(GyrDlp::Hz196)
///     .initialize_9dof().await?;
/// ```
pub struct IcmBuilder<DEVICE, DELAY> {
    device: DEVICE,
    delay: DELAY,
    config: Config,
}

/// An initialized IMU. Use the [`IcmBuilder`] to construct a new instance.
pub struct Icm20948<DEVICE, MAG> {
    device: DEVICE,
    config: Config,
    user_bank: UserBank,
    mag_state: PhantomData<MAG>,
}

impl<BUS, DELAY> IcmBuilder<I2cDevice<BUS>, DELAY>
where
    BUS: I2c,
    DELAY: DelayNs,
{
    /// Creates an uninitialized IMU struct with a default config.
    #[must_use]
    pub fn new_i2c(bus: BUS, delay: DELAY) -> IcmBuilder<I2cDevice<BUS>, DELAY> {
        Self {
            device: I2cDevice {
                bus_inner: bus,
                address: I2cAddress::default(),
            },
            delay,
            config: Config::default(),
        }
    }

    /// Set I2C address of ICM module. See `I2cAddress` for defaults, otherwise `u8` implements `Into<I2cAddress>`
    ///
    /// **Note:** This will not change the actual address, only the address used by this driver.
    #[must_use]
    pub fn set_address(self, address: impl Into<I2cAddress>) -> IcmBuilder<I2cDevice<BUS>, DELAY> {
        IcmBuilder {
            device: I2cDevice {
                address: address.into(),
                ..self.device
            },
            ..self
        }
    }
}

impl<DEVICE, DELAY> IcmBuilder<SpiDevice<DEVICE>, DELAY>
where
    DEVICE: spi::SpiDevice,
    DELAY: DelayNs,
{
    /// Creates an uninitialized IMU struct with a default config.
    #[must_use]
    pub fn new_spi(device: DEVICE, delay: DELAY) -> IcmBuilder<SpiDevice<DEVICE>, DELAY> {
        Self {
            device: SpiDevice { inner: device },
            delay,
            config: Config::default(),
        }
    }
}

impl<DEVICE, DELAY> IcmBuilder<DEVICE, DELAY>
where
    DEVICE: RegisterDevice,
    DELAY: DelayNs,
{
    /// Set the whole IMU config at once
    #[must_use]
    pub fn with_config(self, config: Config) -> IcmBuilder<DEVICE, DELAY> {
        IcmBuilder { config, ..self }
    }

    /// Set accelerometer measuring range, choises are 2G, 4G, 8G or 16G
    #[must_use]
    pub fn acc_range(self, acc_range: AccRange) -> IcmBuilder<DEVICE, DELAY> {
        IcmBuilder {
            config: Config {
                acc_range,
                ..self.config
            },
            ..self
        }
    }

    /// Set accelerometer digital lowpass filter frequency
    #[must_use]
    pub fn acc_dlp(self, acc_dlp: AccDlp) -> IcmBuilder<DEVICE, DELAY> {
        IcmBuilder {
            config: Config {
                acc_dlp,
                ..self.config
            },
            ..self
        }
    }

    /// Set returned unit of accelerometer measurement, choises are Gs or m/s^2
    #[must_use]
    pub fn acc_unit(self, acc_unit: AccUnit) -> IcmBuilder<DEVICE, DELAY> {
        IcmBuilder {
            config: Config {
                acc_unit,
                ..self.config
            },
            ..self
        }
    }

    /// Set accelerometer output data rate
    #[must_use]
    pub fn acc_odr(self, acc_odr: u16) -> IcmBuilder<DEVICE, DELAY> {
        IcmBuilder {
            config: Config {
                acc_odr,
                ..self.config
            },
            ..self
        }
    }

    /// Set gyroscope measuring range, choises are 250Dps, 500Dps, 1000Dps and 2000Dps
    #[must_use]
    pub fn gyr_range(self, gyr_range: GyrRange) -> IcmBuilder<DEVICE, DELAY> {
        IcmBuilder {
            config: Config {
                gyr_range,
                ..self.config
            },
            ..self
        }
    }

    /// Set gyroscope digital low pass filter frequency
    #[must_use]
    pub fn gyr_dlp(self, gyr_dlp: GyrDlp) -> IcmBuilder<DEVICE, DELAY> {
        IcmBuilder {
            config: Config {
                gyr_dlp,
                ..self.config
            },
            ..self
        }
    }

    /// Set returned unit of gyroscope measurement, choises are degrees/s or radians/s
    #[must_use]
    pub fn gyr_unit(self, gyr_unit: GyrUnit) -> IcmBuilder<DEVICE, DELAY> {
        IcmBuilder {
            config: Config {
                gyr_unit,
                ..self.config
            },
            ..self
        }
    }

    /// Set gyroscope output data rate
    #[must_use]
    pub fn gyr_odr(self, gyr_odr: u8) -> IcmBuilder<DEVICE, DELAY> {
        IcmBuilder {
            config: Config {
                gyr_odr,
                ..self.config
            },
            ..self
        }
    }

    /// Initializes the IMU with accelerometer and gyroscope
    pub async fn initialize_6dof(
        mut self,
    ) -> Result<Icm20948<DEVICE, MagDisabled>, SetupError<DEVICE::Error>> {
        let mut imu = Icm20948 {
            device: self.device,
            config: self.config,
            user_bank: UserBank::Bank0,
            mag_state: PhantomData,
        };

        imu.setup_acc_gyr(&mut self.delay).await?;

        Ok(imu)
    }

    /// Initializes the IMU with accelerometer, gyroscope and magnetometer
    pub async fn initialize_9dof(
        mut self,
    ) -> Result<Icm20948<DEVICE, MagEnabled>, SetupError<DEVICE::Error>> {
        let mut imu = Icm20948 {
            device: self.device,
            config: self.config,
            user_bank: UserBank::Bank0,
            mag_state: PhantomData,
        };

        imu.setup_acc_gyr(&mut self.delay).await?;
        imu.setup_mag(&mut self.delay).await?;

        Ok(imu)
    }
}

impl<DEVICE, MAG> Icm20948<DEVICE, MAG>
where
    DEVICE: RegisterDevice,
{
    /// Setup accelerometer and gyroscope according to config
    async fn setup_acc_gyr(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<(), SetupError<DEVICE::Error>> {
        // Ensure known-good state
        self.device_reset(delay).await?;

        // Initially set user bank by force, and check identity
        self.set_user_bank::<Bank0>(true).await?;
        let [imu_whoami] = self.read_from(Bank0::WhoAmI).await?;

        if imu_whoami != IMU_WHOAMI {
            return Err(SetupError::ImuWhoAmI(imu_whoami));
        }

        // Disable sleep mode and set to auto select clock source
        self.write_to(Bank0::PwrMgmt1, 0x01).await?;

        // Set gyro and accel ranges
        self.set_acc_range(self.config.acc_range).await?;
        self.set_gyr_range(self.config.gyr_range).await?;

        // Apply digital lowpass filter settings
        self.set_gyr_dlp(self.config.gyr_dlp).await?;
        self.set_acc_dlp(self.config.acc_dlp).await?;

        // Set gyro and accel output data rate
        self.set_acc_odr(self.config.acc_odr).await?;
        self.set_gyr_odr(self.config.gyr_odr).await?;

        Ok(())
    }

    /// Reset accelerometer / gyroscope module
    pub async fn device_reset(&mut self, delay: &mut impl DelayNs) -> Result<(), DEVICE::Error> {
        delay.delay_ms(20).await;
        self.write_to_flag(Bank0::PwrMgmt1, 1 << 7, 1 << 7).await?;
        delay.delay_ms(50).await;
        Ok(())
    }

    /// Enables main ICM module to act as I2C master (eg. for magnetometer)
    async fn enable_i2c_master(&mut self, enable: bool) -> Result<(), DEVICE::Error> {
        self.write_to_flag(Bank0::UserCtrl, u8::from(enable) << 5, 1 << 5)
            .await
    }

    /// Resets I2C master module
    async fn reset_i2c_master(&mut self) -> Result<(), DEVICE::Error> {
        self.write_to_flag(Bank0::UserCtrl, 1 << 1, 1 << 1).await
    }

    /// Ensure correct user bank for given register
    async fn set_user_bank<R: Register>(&mut self, force: bool) -> Result<(), DEVICE::Error> {
        if (self.user_bank != R::bank()) || force {
            self.device
                .write_registers(REG_BANK_SEL, &[(R::bank() as u8) << 4])
                .await?;
            self.user_bank = R::bank();
        }
        Ok(())
    }

    /// Read a const number `N` of bytes from the requested register
    async fn read_from<const N: usize, R: Register>(
        &mut self,
        reg: R,
    ) -> Result<[u8; N], DEVICE::Error> {
        let mut buf = [0u8; N];
        self.set_user_bank::<R>(false).await?;
        self.device.read_registers(reg.addr(), &mut buf).await?;
        Ok(buf)
    }

    /// Write a single byte to the requeste register
    async fn write_to<R: Register>(&mut self, reg: R, data: u8) -> Result<(), DEVICE::Error> {
        self.set_user_bank::<R>(false).await?;
        self.device.write_registers(reg.addr(), &[data]).await
    }

    /// Write to a register, but only overwrite the parts corresponding to the flag byte
    async fn write_to_flag<R: Register>(
        &mut self,
        reg: R,
        data: u8,
        flag: u8,
    ) -> Result<(), DEVICE::Error> {
        let [mut register] = self.read_from(reg).await?;
        register = (register & !flag) | (data & flag);
        self.write_to(reg, register).await
    }

    /// Put the magnetometer into read mode
    async fn set_mag_read(&mut self) -> Result<(), DEVICE::Error> {
        let [mut reg] = self.read_from(Bank3::I2cSlv0Addr).await?;
        reg &= 0b0111_1111;
        reg |= 1 << 7;
        self.write_to(Bank3::I2cSlv0Addr, reg).await
    }

    /// Put the magnetometer into write mode
    async fn set_mag_write(&mut self) -> Result<(), DEVICE::Error> {
        let [mut reg] = self.read_from(Bank3::I2cSlv0Addr).await?;
        reg &= 0b0111_1111;
        self.write_to(Bank3::I2cSlv0Addr, reg).await
    }

    /// Write `data` to the magnetometer module in `reg` (20 ms non-blocking delays)
    async fn mag_write_to(
        &mut self,
        reg: MagBank,
        data: u8,
        delay: &mut impl DelayNs,
    ) -> Result<(), DEVICE::Error> {
        self.set_mag_write().await?;
        delay.delay_ms(10).await;
        self.write_to(Bank3::I2cSlv0Reg, reg.reg()).await?;
        self.write_to(Bank3::I2cSlv0Do, data).await?;
        self.write_to(Bank3::I2cSlv0Ctrl, (1 << 7) | 1).await?;
        delay.delay_ms(10).await;
        self.set_mag_read().await
    }

    /// Read a `N` bytes from the magnetometer in `reg` (20 ms non-blocking delays)
    async fn mag_read_from<const N: usize>(
        &mut self,
        reg: MagBank,
        delay: &mut impl DelayNs,
    ) -> Result<[u8; N], DEVICE::Error> {
        self.set_mag_read().await?;
        delay.delay_ms(10).await;
        self.write_to(Bank3::I2cSlv0Reg, reg.reg()).await?;
        let data = (1 << 7) | (N as u8).min(16);
        self.write_to(Bank3::I2cSlv0Ctrl, data).await?;
        delay.delay_ms(10).await;
        self.read_from(Bank0::ExtSlvSensData00).await
    }

    /// Configure acceleromter to measure with given range
    pub async fn set_acc_range(&mut self, range: AccRange) -> Result<(), DEVICE::Error> {
        self.write_to_flag(Bank2::AccelConfig, (range as u8) << 1, 0b0110)
            .await?;
        self.config.acc_range = range;
        Ok(())
    }

    /// Configure gyroscope to measure with given range
    pub async fn set_gyr_range(&mut self, range: GyrRange) -> Result<(), DEVICE::Error> {
        self.write_to_flag(Bank2::GyroConfig1, (range as u8) << 1, 0b0110)
            .await?;
        self.config.gyr_range = range;
        Ok(())
    }

    /// Set returned unit of accelerometer
    pub fn set_acc_unit(&mut self, unit: AccUnit) {
        self.config.acc_unit = unit;
    }

    /// Set returned unit of gyroscope
    pub fn set_gyr_unit(&mut self, unit: GyrUnit) {
        self.config.gyr_unit = unit;
    }

    /// Set (or disable) accelerometer digital low-pass filter
    pub async fn set_acc_dlp(&mut self, acc_dlp: AccDlp) -> Result<(), DEVICE::Error> {
        let flag = 0b0011_1001;
        let data = if AccDlp::Disabled != acc_dlp {
            ((acc_dlp as u8) << 3) | 1
        } else {
            0u8
        };
        self.write_to_flag(Bank2::AccelConfig, data, flag).await
    }

    /// Set (or disable) gyroscope digital low-pass filter
    pub async fn set_gyr_dlp(&mut self, gyr_dlp: GyrDlp) -> Result<(), DEVICE::Error> {
        let flag = 0b0011_1001;
        if GyrDlp::Disabled == gyr_dlp {
            self.write_to_flag(Bank2::GyroConfig1, 0u8, flag).await
        } else {
            let data = ((gyr_dlp as u8) << 3) | 1;
            self.write_to_flag(Bank2::GyroConfig1, data, flag).await
        }
    }

    /// Set accelerometer output data rate. Value will be clamped above 4095.
    pub async fn set_acc_odr(&mut self, acc_odr: u16) -> Result<(), DEVICE::Error> {
        let [msb, lsb] = acc_odr.clamp(0, 0xFFF).to_be_bytes();
        self.write_to(Bank2::AccelSmplrtDiv1, msb).await?;
        self.write_to(Bank2::AccelSmplrtDiv2, lsb).await
    }

    /// Set gyroscope output data rate.
    pub async fn set_gyr_odr(&mut self, gyr_odr: u8) -> Result<(), DEVICE::Error> {
        self.write_to(Bank2::GyroSmplrtDiv, gyr_odr).await
    }
}

impl<DEVICE> Icm20948<DEVICE, MagEnabled>
where
    DEVICE: RegisterDevice,
{
    /// Setup magnetometer in continuous mode
    async fn setup_mag(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<(), SetupError<DEVICE::Error>> {
        // Ensure known-good state
        self.mag_reset(delay).await?;

        // Setup magnetometer (i2c slave) clock (default 400 kHz)
        self.write_to(Bank3::I2cMstCtrl, 0x07).await?;

        // Enable I2C master mode for magnetometer
        self.enable_i2c_master(true).await?;

        // Configure slave address as magnetometer
        self.write_to(Bank3::I2cSlv0Addr, MAGNET_ADDR).await?;

        // Verify magnetometer identifier
        let [mag_whoami] = self.mag_read_from(MagBank::DeviceId, delay).await?;

        if mag_whoami != MAG_WHOAMI {
            return Err(SetupError::MagWhoAmI(mag_whoami));
        }

        // Reset magnetometer
        self.mag_reset(delay).await?;

        // Set magnetometer to continuous mode 4 (100 Hz)
        self.mag_write_to(MagBank::Control2, 0b01000, delay).await?;

        // Set slave register to read from
        self.write_to(Bank3::I2cSlv0Reg, MagBank::XDataLow.reg())
            .await?;

        // Set expected read size
        self.write_to(Bank3::I2cSlv0Ctrl, (1 << 7) | 8).await?;

        Ok(())
    }

    /// Reset magnetometer module ( 120 ms non-blocking delays)
    async fn mag_reset(&mut self, delay: &mut impl DelayNs) -> Result<(), DEVICE::Error> {
        // Control 3 register bit 1 resets magnetometer unit
        self.mag_write_to(MagBank::Control3, 1, delay).await?;
        delay.delay_ms(100).await;

        // Reset i2c master module
        self.reset_i2c_master().await
    }

    /// Get vector of scaled magnetometer values
    pub async fn read_mag(&mut self) -> Result<[f32; 3], DEVICE::Error> {
        let mag = self.read_mag_unscaled().await?;
        let mag = mag.map(|x| 0.15 * x as f32);

        Ok(mag)
    }

    /// Get array of unscaled accelerometer values
    pub async fn read_mag_unscaled(&mut self) -> Result<[i16; 3], DEVICE::Error> {
        let raw: [u8; 6] = self.read_from(Bank0::ExtSlvSensData00).await?;
        let mag = collect_3xi16_mag(raw);

        Ok(mag)
    }

    /// Get scaled measurement for accelerometer, gyroscope and magnetometer, and temperature
    pub async fn read_9dof(&mut self) -> Result<Data9Dof<f32>, DEVICE::Error> {
        let raw: [u8; 20] = self.read_from(Bank0::AccelXoutH).await?;
        let [axh, axl, ayh, ayl, azh, azl, gxh, gxl, gyh, gyl, gzh, gzl, tph, tpl, mxl, mxh, myl, myh, mzl, mzh] =
            raw;

        let acc = self.scaled_acc_from_bytes([axh, axl, ayh, ayl, azh, azl]);
        let gyr = self.scaled_gyr_from_bytes([gxh, gxl, gyh, gyl, gzh, gzl]);
        let mag = self.scaled_mag_from_bytes([mxl, mxh, myl, myh, mzl, mzh]);

        let tmp = self.scaled_tmp_from_bytes([tph, tpl]);

        Ok(Data9Dof { acc, gyr, mag, tmp })
    }

    /// Get unscaled measurements for accelerometer and gyroscope, and temperature
    pub async fn read_9dof_unscaled(&mut self) -> Result<Data9Dof<i16>, DEVICE::Error> {
        let raw: [u8; 20] = self.read_from(Bank0::AccelXoutH).await?;
        let [axh, axl, ayh, ayl, azh, azl, gxh, gxl, gyh, gyl, gzh, gzl, tph, tpl, mxl, mxh, myl, myh, mzl, mzh] =
            raw;

        let acc = collect_3xi16([axh, axl, ayh, ayl, azh, azl]);
        let gyr = collect_3xi16([gxh, gxl, gyh, gyl, gzh, gzl]);
        let mag = collect_3xi16_mag([mxl, mxh, myl, myh, mzl, mzh]);

        let tmp = i16::from_be_bytes([tph, tpl]);

        Ok(Data9Dof { acc, gyr, mag, tmp })
    }

    /// Takes 6 bytes converts them into a Vector3 of floats, unit is micro tesla
    fn scaled_mag_from_bytes(&self, bytes: [u8; 6]) -> [f32; 3] {
        collect_3xi16_mag(bytes).map(|x| 0.15 * x as f32)
    }
}

impl<DEVICE, MAG> Icm20948<DEVICE, MAG>
where
    DEVICE: RegisterDevice,
{
    /// Takes 6 bytes converts them into a Vector3 of floats
    fn scaled_acc_from_bytes(&self, bytes: [u8; 6]) -> [f32; 3] {
        collect_3xi16(bytes).map(|x| f32::from(x) * self.acc_scalar())
    }

    /// Takes 6 bytes converts them into a Vector3 of floats
    fn scaled_gyr_from_bytes(&self, bytes: [u8; 6]) -> [f32; 3] {
        collect_3xi16(bytes).map(|x| f32::from(x) * self.gyr_scalar())
    }

    /// Takes 2 bytes converts them into a temerature as a float
    fn scaled_tmp_from_bytes(&self, bytes: [u8; 2]) -> f32 {
        f32::from(i16::from_be_bytes(bytes)) / 333.87 + 21.
    }

    /// Get array of unscaled accelerometer values
    pub async fn read_acc_unscaled(&mut self) -> Result<[i16; 3], DEVICE::Error> {
        let raw = self.read_from(Bank0::AccelXoutH).await?;
        Ok(collect_3xi16(raw))
    }

    /// Get array of scaled accelerometer values
    pub async fn read_acc(&mut self) -> Result<[f32; 3], DEVICE::Error> {
        let acc = self
            .read_acc_unscaled()
            .await?
            .map(|x| f32::from(x) * self.acc_scalar());
        Ok(acc)
    }

    /// Get array of unscaled gyroscope values
    pub async fn read_gyr_unscaled(&mut self) -> Result<[i16; 3], DEVICE::Error> {
        let raw = self.read_from(Bank0::GyroXoutH).await?;
        Ok(collect_3xi16(raw))
    }

    /// Get array of scaled gyroscope values
    pub async fn read_gyr(&mut self) -> Result<[f32; 3], DEVICE::Error> {
        let gyr = self
            .read_gyr_unscaled()
            .await?
            .map(|x| f32::from(x) * self.gyr_scalar());
        Ok(gyr)
    }

    /// Get scaled measurements for accelerometer and gyroscope, and temperature
    pub async fn read_6dof(&mut self) -> Result<Data6Dof<f32>, DEVICE::Error> {
        let raw: [u8; 14] = self.read_from(Bank0::AccelXoutH).await?;
        let [axh, axl, ayh, ayl, azh, azl, gxh, gxl, gyh, gyl, gzh, gzl, tph, tpl] = raw;

        let acc = self.scaled_acc_from_bytes([axh, axl, ayh, ayl, azh, azl]);
        let gyr = self.scaled_gyr_from_bytes([gxh, gxl, gyh, gyl, gzh, gzl]);

        let tmp = self.scaled_tmp_from_bytes([tph, tpl]);

        Ok(Data6Dof { acc, gyr, tmp })
    }

    /// Get unscaled measurements for accelerometer and gyroscope, and temperature
    pub async fn read_6dof_unscaled(&mut self) -> Result<Data6Dof<i16>, DEVICE::Error> {
        let raw: [u8; 14] = self.read_from(Bank0::AccelXoutH).await?;
        let [axh, axl, ayh, ayl, azh, azl, gxh, gxl, gyh, gyl, gzh, gzl, tph, tpl] = raw;

        let acc = collect_3xi16([axh, axl, ayh, ayl, azh, azl]);
        let gyr = collect_3xi16([gxh, gxl, gyh, gyl, gzh, gzl]);

        let tmp = i16::from_be_bytes([tph, tpl]);

        Ok(Data6Dof { acc, gyr, tmp })
    }

    /// Set gyroscope calibration offsets by writing them to the IMU
    pub async fn set_gyr_offsets(&mut self, offsets: [i16; 3]) -> Result<(), DEVICE::Error> {
        let [[xh, xl], [yh, yl], [zh, zl]]: [[u8; 2]; 3] = offsets.map(|x| (-x).to_be_bytes());

        self.set_user_bank::<Bank2>(false).await?;

        self.device
            .write_registers(Bank2::XgOffsH.addr(), &[xh, xl])
            .await?;
        self.device
            .write_registers(Bank2::YgOffsH.addr(), &[yh, yl])
            .await?;
        self.device
            .write_registers(Bank2::ZgOffsH.addr(), &[zh, zl])
            .await?;

        Ok(())
    }

    /// Set accelerometer calibration offsets by writing them to the IMU
    pub async fn set_acc_offsets(&mut self, offsets: [i16; 3]) -> Result<(), DEVICE::Error> {
        let [[xh, xl], [yh, yl], [zh, zl]]: [[u8; 2]; 3] = offsets.map(|x| (-x).to_be_bytes());

        self.set_user_bank::<Bank1>(false).await?;

        self.device
            .write_registers(Bank1::XaOffsH.addr(), &[xh, xl])
            .await?;
        self.device
            .write_registers(Bank1::YaOffsH.addr(), &[yh, yl])
            .await?;
        self.device
            .write_registers(Bank1::ZaOffsH.addr(), &[zh, zl])
            .await?;

        Ok(())
    }

    /// Returns the number of new readings in FIFO buffer
    pub async fn new_data_ready(&mut self) -> u8 {
        self.read_from(Bank0::DataRdyStatus)
            .await
            .map_or(0, |[b]| b & 0b1111)
    }

    /// Returns the scalar corresponding to the unit and range configured
    pub const fn acc_scalar(&self) -> f32 {
        self.config.acc_unit.scalar() / self.config.acc_range.divisor()
    }

    /// Returns the scalar corresponding to the unit and range configured
    pub const fn gyr_scalar(&self) -> f32 {
        self.config.gyr_unit.scalar() / self.config.gyr_range.divisor()
    }
}

/// Collects 6 bytes into a vector of i16 values (acc/gyr only)
const fn collect_3xi16(values: [u8; 6]) -> [i16; 3] {
    let [xh, xl, yh, yl, zh, zl] = values;
    [
        i16::from_be_bytes([xh, xl]),
        i16::from_be_bytes([yh, yl]),
        i16::from_be_bytes([zh, zl]),
    ]
}

/// Collects 6 bytes into a vector of i16 values (mag only)
const fn collect_3xi16_mag(values: [u8; 6]) -> [i16; 3] {
    let [xl, xh, yl, yh, zl, zh] = values;

    #[cfg(feature = "align-mag")]
    let mag = [
        i16::from_be_bytes([xh, xl]),
        -i16::from_be_bytes([yh, yl]),
        -i16::from_be_bytes([zh, zl]),
    ];

    #[cfg(not(feature = "align-mag"))]
    let mag = [
        i16::from_be_bytes([xh, xl]),
        i16::from_be_bytes([yh, yl]),
        i16::from_be_bytes([zh, zl]),
    ];

    mag
}
