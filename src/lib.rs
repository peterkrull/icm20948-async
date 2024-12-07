#![no_std]

use core::{future::Future, marker::PhantomData};
use embedded_hal_async::{delay::DelayNs, digital::Wait, i2c::I2c, spi::SpiDevice};
use nalgebra::Vector3;

mod reg;
use crate::reg::*;

const MAGNET_ADDR: u8 = 0x0C; // I2C address of magnetometer

#[derive(Clone, Copy)]
/// Container for accelerometer and gyroscope measurements
pub struct Data6Dof<T> {
    pub acc: Vector3<T>,
    pub gyr: Vector3<T>,
    pub tmp: T,
}

#[derive(Clone, Copy)]
/// Container for accelerometer, gyroscope and magnetometer measurements
pub struct Data9Dof<T> {
    pub acc: Vector3<T>,
    pub gyr: Vector3<T>,
    pub mag: Vector3<T>,
    pub tmp: T,
}

// Compile-time MAG states
pub struct MagEnabled {
    is_calibrated: bool,
    offset: Vector3<f32>,
    scale: Vector3<f32>,
}

impl MagEnabled {
    fn uncralibrated() -> Self {
        Self {
            is_calibrated: false,
            offset: Vector3::zeros(),
            scale: Vector3::from_element(1.),
        }
    }
}
pub struct MagDisabled;

// Compile-time init states
pub struct Init;
pub struct NotInit;

// Type to hold bus information for I2c
pub struct IcmBusI2c<I2C> {
    bus_inner: I2C,
    address: I2cAddress,
}
// Type to hold bus information for Spi
pub struct IcmBusSpi<SPI> {
    bus_inner: SPI,
}

// Trait to allow for generic behavior across I2c or Spi usage
#[allow(async_fn_in_trait)]
pub trait BusTransfer {
    type Error;
    type Inner;
    fn destroy(self) -> Self::Inner;
    async fn bus_transfer(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error>;
    async fn bus_write(&mut self, write: &[u8]) -> Result<(), Self::Error>;
}

// Implementation of bus trait for I2c
impl<I2C, E> BusTransfer for IcmBusI2c<I2C>
where
    I2C: I2c<Error = E>,
    E: Into<IcmError<E>>,
{
    type Inner = I2C;
    type Error = E;

    fn destroy(self) -> Self::Inner {
        self.bus_inner
    }

    async fn bus_transfer(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), E> {
        self.bus_inner
            .write_read(self.address.get(), write, read)
            .await
    }

    async fn bus_write(&mut self, write: &[u8]) -> Result<(), E> {
        self.bus_inner.write(self.address.get(), write).await
    }
}

// Implementation of bus trait for Spi
impl<SPI, E> BusTransfer for IcmBusSpi<SPI>
where
    SPI: SpiDevice<Error = E>,
    E: Into<IcmError<E>>,
{
    type Inner = SPI;
    type Error = E;

    fn destroy(self) -> Self::Inner {
        self.bus_inner
    }

    async fn bus_transfer(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), E> {
        self.bus_inner.transfer(read, write).await
    }

    async fn bus_write(&mut self, write: &[u8]) -> Result<(), E> {
        self.bus_inner.write(write).await
    }
}

pub struct Icm20948<BUS, MAG, INIT, DELAY> {
    bus: BUS,
    config: Icm20948Config,
    user_bank: UserBank,
    delay: DELAY,
    mag_state: MAG,
    init_state: PhantomData<INIT>,
}

impl<BUS, DELAY> Icm20948<IcmBusI2c<BUS>, MagDisabled, NotInit, DELAY>
where
    BUS: I2c,
    DELAY: DelayNs,
{
    /// Creates an uninitialized IMU struct with the given config.
    #[must_use]
    pub fn new_i2c_from_cfg(
        bus: BUS,
        cfg: Icm20948Config,
        delay: DELAY,
    ) -> Icm20948<IcmBusI2c<BUS>, MagDisabled, NotInit, DELAY> {
        Self {
            bus: IcmBusI2c {
                bus_inner: bus,
                address: I2cAddress::default(),
            },
            config: cfg,
            user_bank: UserBank::Bank0,
            delay,
            mag_state: MagDisabled,
            init_state: PhantomData::<NotInit>,
        }
    }

    /// Creates an uninitialized IMU struct with a default config.
    #[must_use]
    pub fn new_i2c(
        bus: BUS,
        delay: DELAY,
    ) -> Icm20948<IcmBusI2c<BUS>, MagDisabled, NotInit, DELAY> {
        Self::new_i2c_from_cfg(bus, Icm20948Config::default(), delay)
    }
}

impl<BUS, DELAY> Icm20948<IcmBusSpi<BUS>, MagDisabled, NotInit, DELAY>
where
    BUS: SpiDevice,
    DELAY: DelayNs,
{
    /// Creates an uninitialized IMU struct with the given config.
    #[must_use]
    pub fn new_spi_from_cfg(
        bus: BUS,
        cfg: Icm20948Config,
        delay: DELAY,
    ) -> Icm20948<IcmBusSpi<BUS>, MagDisabled, NotInit, DELAY> {
        Self {
            bus: IcmBusSpi { bus_inner: bus },
            config: cfg,
            user_bank: UserBank::Bank0,
            mag_state: MagDisabled,
            delay,
            init_state: PhantomData::<NotInit>,
        }
    }

    /// Creates an uninitialized IMU struct with a default config.
    #[must_use]
    pub fn new_spi(
        bus: BUS,
        delay: DELAY,
    ) -> Icm20948<IcmBusSpi<BUS>, MagDisabled, NotInit, DELAY> {
        Self::new_spi_from_cfg(bus, Icm20948Config::default(), delay)
    }
}

impl<BUS: BusTransfer, MAG, INIT, DELAY> Icm20948<BUS, MAG, INIT, DELAY> {
    /// Consumes the `Icm20948` and releases the bus back to the user
    #[must_use]
    pub fn destroy(self) -> BUS::Inner {
        self.bus.destroy()
    }
}

impl<BUS, DELAY> Icm20948<IcmBusI2c<BUS>, MagDisabled, NotInit, DELAY>
where
    BUS: I2c,
    DELAY: DelayNs,
{
    /// Set I2C address of ICM module. See `I2cAddress` for defaults, otherwise `u8` implements `Into<I2cAddress>`
    #[must_use]
    pub fn set_address(
        self,
        address: impl Into<I2cAddress>,
    ) -> Icm20948<IcmBusI2c<BUS>, MagDisabled, NotInit, DELAY> {
        Icm20948 {
            bus: IcmBusI2c {
                address: address.into(),
                ..self.bus
            },
            ..self
        }
    }
}

impl<BUS, DELAY> Icm20948<BUS, MagDisabled, NotInit, DELAY>
where
    BUS: BusTransfer,
    DELAY: DelayNs,
{
    /*
        Configuration methods
    */

    /// Set accelerometer measuring range, choises are 2G, 4G, 8G or 16G
    #[must_use]
    pub fn acc_range(self, acc_range: AccRange) -> Icm20948<BUS, MagDisabled, NotInit, DELAY> {
        Icm20948 {
            config: Icm20948Config {
                acc_range,
                ..self.config
            },
            ..self
        }
    }

    /// Set accelerometer digital lowpass filter frequency
    #[must_use]
    pub fn acc_dlp(self, acc_dlp: AccDlp) -> Icm20948<BUS, MagDisabled, NotInit, DELAY> {
        Icm20948 {
            config: Icm20948Config {
                acc_dlp,
                ..self.config
            },
            ..self
        }
    }

    /// Set returned unit of accelerometer measurement, choises are Gs or m/s^2
    #[must_use]
    pub fn acc_unit(self, acc_unit: AccUnit) -> Icm20948<BUS, MagDisabled, NotInit, DELAY> {
        Icm20948 {
            config: Icm20948Config {
                acc_unit,
                ..self.config
            },
            ..self
        }
    }

    /// Set accelerometer output data rate
    #[must_use]
    pub fn acc_odr(self, acc_odr: u16) -> Icm20948<BUS, MagDisabled, NotInit, DELAY> {
        Icm20948 {
            config: Icm20948Config {
                acc_odr,
                ..self.config
            },
            ..self
        }
    }

    /// Set gyroscope measuring range, choises are 250Dps, 500Dps, 1000Dps and 2000Dps
    #[must_use]
    pub fn gyr_range(self, gyr_range: GyrRange) -> Icm20948<BUS, MagDisabled, NotInit, DELAY> {
        Icm20948 {
            config: Icm20948Config {
                gyr_range,
                ..self.config
            },
            ..self
        }
    }

    /// Set gyroscope digital low pass filter frequency
    #[must_use]
    pub fn gyr_dlp(self, gyr_dlp: GyrDlp) -> Icm20948<BUS, MagDisabled, NotInit, DELAY> {
        Icm20948 {
            config: Icm20948Config {
                gyr_dlp,
                ..self.config
            },
            ..self
        }
    }

    /// Set returned unit of gyroscope measurement, choises are degrees/s or radians/s
    #[must_use]
    pub fn gyr_unit(self, gyr_unit: GyrUnit) -> Icm20948<BUS, MagDisabled, NotInit, DELAY> {
        Icm20948 {
            config: Icm20948Config {
                gyr_unit,
                ..self.config
            },
            ..self
        }
    }

    /// Set gyroscope output data rate
    #[must_use]
    pub fn gyr_odr(self, gyr_odr: u8) -> Icm20948<BUS, MagDisabled, NotInit, DELAY> {
        Icm20948 {
            config: Icm20948Config {
                gyr_odr,
                ..self.config
            },
            ..self
        }
    }

    /// Set gyroscope output data rate
    #[must_use]
    pub fn interrupt(self, int: impl Into<Option<Interrupt>>) -> Icm20948<BUS, MagDisabled, NotInit, DELAY> {
        Icm20948 {
            config: Icm20948Config {
                int: int.into(),
                ..self.config
            },
            ..self
        }
    }

    /*
        Initialization methods
    */

    /// Initializes the IMU with accelerometer and gyroscope
    pub async fn initialize_6dof(
        mut self,
    ) -> Result<Icm20948<BUS, MagDisabled, Init, DELAY>, IcmError<BUS::Error>> {
        self.setup_acc_gyr().await?;
        if let Some(int) = self.config.int {
            self.setup_interrupt(&int).await?;
        }

        Ok(Icm20948 {
            mag_state: MagDisabled,
            init_state: PhantomData::<Init>,
            bus: self.bus,
            config: self.config,
            user_bank: self.user_bank,
            delay: self.delay,
        })
    }

    /// Initializes the IMU with accelerometer, gyroscope and magnetometer
    pub async fn initialize_9dof(
        mut self,
    ) -> Result<Icm20948<BUS, MagEnabled, Init, DELAY>, IcmError<BUS::Error>> {
        self.setup_acc_gyr().await?;
        self.setup_mag().await?;
        if let Some(int) = self.config.int {
            self.setup_interrupt(&int).await?;
        }

        Ok(Icm20948 {
            mag_state: MagEnabled::uncralibrated(),
            init_state: PhantomData::<Init>,
            bus: self.bus,
            config: self.config,
            user_bank: self.user_bank,
            delay: self.delay,
        })
    }

    /// Setup accelerometer and gyroscope according to config
    async fn setup_acc_gyr(&mut self) -> Result<(), IcmError<BUS::Error>> {
        // Ensure known-good state
        self.device_reset().await?;

        // Initially set user bank by force, and check identity
        self.set_user_bank(&Bank0::WhoAmI, true).await?;
        let [whoami] = self.read_from(Bank0::WhoAmI).await?;

        if whoami != 0xEA {
            return Err(IcmError::ImuSetupError);
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

    /// Setup magnetometer in continuous mode
    async fn setup_mag(&mut self) -> Result<(), IcmError<BUS::Error>> {
        // Ensure known-good state
        self.mag_reset().await?;

        // Setup magnetometer (i2c slave) clock (default 400 kHz)
        self.write_to(Bank3::I2cMstCtrl, 0x07).await?;

        // Enable I2C master mode for magnetometer
        self.enable_i2c_master(true).await?;

        // Configure slave address as magnetometer
        self.write_to(Bank3::I2cSlv0Addr, MAGNET_ADDR).await?;

        // Verify magnetometer identifier
        let [whoami] = self.mag_read_from(MagBank::DeviceId).await?;

        if whoami != 9 {
            return Err(IcmError::MagSetupError);
        }

        // Reset magnetometer
        self.mag_reset().await?;

        // Set magnetometer to continuous mode 4 (100 Hz)
        self.mag_write_to(MagBank::Control2.reg(), 0b01000).await?;

        // Set slave register to read from
        self.write_to(Bank3::I2cSlv0Reg, MagBank::XDataLow.reg())
            .await?;

        // Set expected read size
        self.write_to(Bank3::I2cSlv0Ctrl, 1 << 7 | 8).await?;

        Ok(())
    }

    async fn setup_interrupt(&mut self, cfg: &Interrupt) -> Result<(), IcmError<BUS::Error>> {

        let int_pin_cfg =
            (cfg.active_low as u8) << 7 |
            (cfg.open_drain as u8) << 6 |
            (cfg.latch_on as u8) << 5 |
            (cfg.clear_on_read as u8) << 4;

        self.write_to(Bank0::IntPinCfg, int_pin_cfg).await?;

        let int_enable = 
            (cfg.wake_on_motion as u8) << 3 |
            (cfg.pll_ready as u8) << 2 |
            (cfg.dmp_ready as u8) << 1 |
            (cfg.i2c_master as u8) << 0;

        self.write_to(Bank0::IntEnable, int_enable).await?;
        self.write_to(Bank0::IntEnable1, cfg.raw_data_ready as u8).await?;

        Ok(())
    }
}

impl<BUS, MAG, INIT, DELAY> Icm20948<BUS, MAG, INIT, DELAY>
where
    BUS: BusTransfer,
    DELAY: DelayNs,
{
    /// Reset accelerometer / gyroscope module
    pub async fn device_reset(&mut self) -> Result<(), BUS::Error> {
        self.delay.delay_ms(20).await;
        self.write_to_flag(Bank0::PwrMgmt1, 1 << 7, 1 << 7).await?;
        self.delay.delay_ms(50).await;
        Ok(())
    }

    /// Enables main ICM module to act as I2C master (eg. for magnetometer)
    async fn enable_i2c_master(&mut self, enable: bool) -> Result<(), BUS::Error> {
        self.write_to_flag(Bank0::UserCtrl, u8::from(enable) << 5, 1 << 5)
            .await
    }

    /// Resets I2C master module
    async fn reset_i2c_master(&mut self) -> Result<(), BUS::Error> {
        self.write_to_flag(Bank0::UserCtrl, 1 << 1, 1 << 1).await
    }

    /// Ensure correct user bank for given register
    async fn set_user_bank<R: Register + Copy>(&mut self, bank: &R, force: bool) -> Result<(), BUS::Error> {
        if (self.user_bank != bank.bank()) || force {
            self.bus
                .bus_write(&[REG_BANK_SEL, (bank.bank() as u8) << 4])
                .await?;
            self.user_bank = bank.bank();
        }
        Ok(())
    }

    /// Read a const number `N` of bytes from the requested register
    async fn read_from<const N: usize, R: Register + Copy>(
        &mut self,
        cmd: R,
    ) -> Result<[u8; N], BUS::Error> {
        let mut buf = [0u8; N];
        self.set_user_bank(&cmd, false).await?;
        self.bus.bus_transfer(&[cmd.reg()], &mut buf).await?;
        Ok(buf)
    }

    /// Write a single byte to the requeste register
    async fn write_to<R: Register + Copy>(&mut self, cmd: R, data: u8) -> Result<(), BUS::Error> {
        self.set_user_bank(&cmd, false).await?;
        self.bus.bus_write(&[cmd.reg(), data]).await
    }

    /// Write to a register, but only overwrite the parts corresponding to the flag byte
    async fn write_to_flag<R>(&mut self, cmd: R, data: u8, flag: u8) -> Result<(), BUS::Error>
    where
        R: Register + Copy + Clone,
    {
        let [mut register] = self.read_from(cmd).await?;
        register = (register & !flag) | (data & flag);
        self.write_to(cmd, register).await
    }

    /// Put the magnetometer into read mode
    async fn set_mag_read(&mut self) -> Result<(), BUS::Error> {
        let [mut reg] = self.read_from(Bank3::I2cSlv0Addr).await?;
        reg &= 0b0111_1111;
        reg |= 1 << 7;
        self.write_to(Bank3::I2cSlv0Addr, reg).await
    }

    /// Put the magnetometer into write mode
    async fn set_mag_write(&mut self) -> Result<(), BUS::Error> {
        let [mut reg] = self.read_from(Bank3::I2cSlv0Addr).await?;
        reg &= 0b0111_1111;
        self.write_to(Bank3::I2cSlv0Addr, reg).await
    }

    /// Write `data` to the magnetometer module in `reg` (20 ms non-blocking delays)
    async fn mag_write_to(&mut self, reg: u8, data: u8) -> Result<(), BUS::Error> {
        self.set_mag_write().await?;
        self.delay.delay_ms(10).await;
        self.write_to(Bank3::I2cSlv0Reg, reg).await?;
        self.write_to(Bank3::I2cSlv0Do, data).await?;
        self.write_to(Bank3::I2cSlv0Ctrl, 1 << 7 | 1).await?;
        self.delay.delay_ms(10).await;
        self.set_mag_read().await
    }

    /// Read a `N` bytes from the magnetometer in `reg` (20 ms non-blocking delays)
    async fn mag_read_from<const N: usize>(&mut self, reg: MagBank) -> Result<[u8; N], BUS::Error> {
        self.set_mag_read().await?;
        self.delay.delay_ms(10).await;
        self.write_to(Bank3::I2cSlv0Reg, reg.reg()).await?;
        self.write_to(Bank3::I2cSlv0Ctrl, 1 << 7 | N as u8).await?;
        self.delay.delay_ms(10).await;
        self.read_from(Bank0::ExtSlvSensData00).await
    }

    /// Reset magnetometer module ( 120 ms non-blocking delays)
    async fn mag_reset(&mut self) -> Result<(), BUS::Error> {
        // Control 3 register bit 1 resets magnetometer unit
        self.mag_write_to(MagBank::Control3.reg(), 1).await?;
        self.delay.delay_ms(100).await;

        // Reset i2c master module
        self.reset_i2c_master().await
    }

    /// Configure acceleromter to measure with given range
    pub async fn set_acc_range(&mut self, range: AccRange) -> Result<(), BUS::Error> {
        self.write_to_flag(Bank2::AccelConfig, (range as u8) << 1, 0b0110)
            .await?;
        self.config.acc_range = range;
        Ok(())
    }

    /// Configure gyroscope to measure with given range
    pub async fn set_gyr_range(&mut self, range: GyrRange) -> Result<(), BUS::Error> {
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
    pub async fn set_acc_dlp(&mut self, acc_dlp: AccDlp) -> Result<(), BUS::Error> {
        if AccDlp::Disabled == acc_dlp {
            self.write_to_flag(Bank2::AccelConfig, 0u8, 0b0011_1001)
                .await
        } else {
            self.write_to_flag(Bank2::AccelConfig, (acc_dlp as u8) << 3 | 1, 0b0011_1001)
                .await
        }
    }

    /// Set (or disable) gyroscope digital low-pass filter
    pub async fn set_gyr_dlp(&mut self, gyr_dlp: GyrDlp) -> Result<(), BUS::Error> {
        if GyrDlp::Disabled == gyr_dlp {
            self.write_to_flag(Bank2::GyroConfig1, 0u8, 0b0011_1001)
                .await
        } else {
            self.write_to_flag(Bank2::GyroConfig1, (gyr_dlp as u8) << 3 | 1, 0b0011_1001)
                .await
        }
    }

    /// Set accelerometer output data rate. Value will be clamped above 4095.
    pub async fn set_acc_odr(&mut self, acc_odr: u16) -> Result<(), BUS::Error> {
        let [msb, lsb] = acc_odr.clamp(0, 0xFFF).to_be_bytes();
        self.write_to(Bank2::AccelSmplrtDiv1, msb).await?;
        self.write_to(Bank2::AccelSmplrtDiv2, lsb).await
    }

    /// Set gyroscope output data rate.
    pub async fn set_gyr_odr(&mut self, gyr_odr: u8) -> Result<(), BUS::Error> {
        self.write_to(Bank2::GyroSmplrtDiv, gyr_odr).await
    }
}

impl<BUS, DELAY> Icm20948<BUS, MagEnabled, Init, DELAY>
where
    BUS: BusTransfer,
    DELAY: DelayNs,
{
    /// Apply the saved calibration offset+scale to measurement vector
    fn apply_mag_calibration(&self, mag: &mut Vector3<f32>) {
        if self.mag_state.is_calibrated {
            *mag = mag.zip_zip_map(&self.mag_state.offset, &self.mag_state.scale, |m, o, s| {
                (m - o) / s
            });
        }
    }

    /// Set magnetometer calibration data (offset,scale)
    pub fn set_mag_calibration(&mut self, offset: [f32; 3], scale: [f32; 3]) {
        self.mag_state.is_calibrated = true;
        self.mag_state.offset = offset.into();
        self.mag_state.scale = scale.into();
    }

    /// Resets (disables) magnetometer calibration data
    pub fn reset_mag_calibration(&mut self) {
        self.mag_state.is_calibrated = false;
        self.mag_state.offset = Vector3::zeros();
        self.mag_state.scale = Vector3::from_element(1.);
    }

    /// Get vector of scaled magnetometer values
    pub async fn read_mag(&mut self) -> Result<Vector3<f32>, BUS::Error> {
        let mut mag = self.read_mag_unscaled().await?.map(|x| (x as f32)).into();
        self.apply_mag_calibration(&mut mag);

        Ok(mag)
    }

    /// Get array of unscaled accelerometer values
    pub async fn read_mag_unscaled(&mut self) -> Result<[i16; 3], BUS::Error> {
        let raw: [u8; 6] = self.read_from(Bank0::ExtSlvSensData00).await?;
        let mag = collect_3xi16_mag(raw);

        Ok(mag)
    }

    /// Get scaled measurement for accelerometer, gyroscope and magnetometer, and temperature
    pub async fn read_9dof(&mut self) -> Result<Data9Dof<f32>, BUS::Error> {
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
    pub async fn read_9dof_unscaled(&mut self) -> Result<Data9Dof<i16>, BUS::Error> {
        let raw: [u8; 20] = self.read_from(Bank0::AccelXoutH).await?;
        let [axh, axl, ayh, ayl, azh, azl, gxh, gxl, gyh, gyl, gzh, gzl, tph, tpl, mxl, mxh, myl, myh, mzl, mzh] =
            raw;

        let acc = collect_3xi16([axh, axl, ayh, ayl, azh, azl]).into();
        let gyr = collect_3xi16([gxh, gxl, gyh, gyl, gzh, gzl]).into();
        let mag = collect_3xi16_mag([mxl, mxh, myl, myh, mzl, mzh]).into();

        let tmp = i16::from_be_bytes([tph, tpl]);

        Ok(Data9Dof { acc, gyr, mag, tmp })
    }

    /// Takes 6 bytes converts them into a Vector3 of floats, unit is micro tesla
    fn scaled_mag_from_bytes(&self, bytes: [u8; 6]) -> Vector3<f32> {
        let mut mag = collect_3xi16_mag(bytes).map(|x| (0.15 * x as f32)).into();
        self.apply_mag_calibration(&mut mag);
        mag
    }
}

impl<BUS, MAG, DELAY> Icm20948<BUS, MAG, Init, DELAY>
where
    BUS: BusTransfer,
    DELAY: DelayNs,
{
    /// Takes 6 bytes converts them into a Vector3 of floats
    fn scaled_acc_from_bytes(&self, bytes: [u8; 6]) -> Vector3<f32> {
        let acc = collect_3xi16(bytes).map(|x| f32::from(x) * self.acc_scalar());
        Vector3::from(acc)
    }

    /// Takes 6 bytes converts them into a Vector3 of floats
    fn scaled_gyr_from_bytes(&self, bytes: [u8; 6]) -> Vector3<f32> {
        let gyr = collect_3xi16(bytes).map(|x| f32::from(x) * self.gyr_scalar());
        Vector3::from(gyr)
    }

    /// Takes 2 bytes converts them into a temerature as a float
    fn scaled_tmp_from_bytes(&self, bytes: [u8; 2]) -> f32 {
        f32::from(i16::from_be_bytes(bytes)) / 333.87 + 21.
    }

    /// Get array of unscaled accelerometer values
    pub async fn read_acc_unscaled(&mut self) -> Result<Vector3<i16>, BUS::Error> {
        let raw = self.read_from(Bank0::AccelXoutH).await?;
        Ok(collect_3xi16(raw).into())
    }

    /// Get array of scaled accelerometer values
    pub async fn read_acc(&mut self) -> Result<Vector3<f32>, BUS::Error> {
        let acc = self
            .read_acc_unscaled()
            .await?
            .map(|x| f32::from(x) * self.acc_scalar());
        Ok(acc)
    }

    /// Get array of unscaled gyroscope values
    pub async fn read_gyr_unscaled(&mut self) -> Result<Vector3<i16>, BUS::Error> {
        let raw = self.read_from(Bank0::GyroXoutH).await?;
        Ok(collect_3xi16(raw).into())
    }

    /// Get array of scaled gyroscope values
    pub async fn read_gyr(&mut self) -> Result<Vector3<f32>, BUS::Error> {
        let gyr = self
            .read_gyr_unscaled()
            .await?
            .map(|x| f32::from(x) * self.gyr_scalar());
        Ok(gyr)
    }

    /// Get scaled measurements for accelerometer and gyroscope, and temperature
    pub async fn read_6dof(&mut self) -> Result<Data6Dof<f32>, BUS::Error> {
        let raw: [u8; 14] = self.read_from(Bank0::AccelXoutH).await?;
        let [axh, axl, ayh, ayl, azh, azl, gxh, gxl, gyh, gyl, gzh, gzl, tph, tpl] = raw;

        let acc = self.scaled_acc_from_bytes([axh, axl, ayh, ayl, azh, azl]);
        let gyr = self.scaled_gyr_from_bytes([gxh, gxl, gyh, gyl, gzh, gzl]);

        let tmp = self.scaled_tmp_from_bytes([tph, tpl]);

        Ok(Data6Dof { acc, gyr, tmp })
    }

    /// Get unscaled measurements for accelerometer and gyroscope, and temperature
    pub async fn read_6dof_unscaled(&mut self) -> Result<Data6Dof<i16>, BUS::Error> {
        let raw: [u8; 14] = self.read_from(Bank0::AccelXoutH).await?;
        let [axh, axl, ayh, ayl, azh, azl, gxh, gxl, gyh, gyl, gzh, gzl, tph, tpl] = raw;

        let acc = collect_3xi16([axh, axl, ayh, ayl, azh, azl]).into();
        let gyr = collect_3xi16([gxh, gxl, gyh, gyl, gzh, gzl]).into();

        let tmp = i16::from_be_bytes([tph, tpl]);

        Ok(Data6Dof { acc, gyr, tmp })
    }

    /// Collects and averages `num` sampels for gyro calibration and saves them on-chip
    pub async fn gyr_calibrate(&mut self, num: usize) -> Result<(), BUS::Error> {
        let mut offset: Vector3<i32> = Vector3::default();
        for _ in 0..num {
            offset += self.read_gyr_unscaled().await?.map(|x| x as i32);
            self.delay.delay_ms(10).await;
        }

        self.set_gyr_offsets(offset.map(|x| { x / num as i32 } as i16))
            .await?;

        Ok(())
    }

    /// Set gyroscope calibration offsets by writing them to the IMU
    pub async fn set_gyr_offsets(&mut self, offsets: Vector3<i16>) -> Result<(), BUS::Error> {
        let [[xh, xl], [yh, yl], [zh, zl]]: [[u8; 2]; 3] =
            offsets.map(|x| (-x).to_be_bytes()).into();

        self.set_user_bank(&Bank2::XgOffsUsrh, false).await?;

        self.bus
            .bus_write(&[Bank2::XgOffsUsrh.reg(), xh, xl])
            .await?;
        self.bus
            .bus_write(&[Bank2::YgOffsUsrh.reg(), yh, yl])
            .await?;
        self.bus
            .bus_write(&[Bank2::ZgOffsUsrh.reg(), zh, zl])
            .await?;

        Ok(())
    }

    /// Set accelerometer calibration offsets by writing them to the IMU
    pub async fn set_acc_offsets(&mut self, offsets: Vector3<i16>) -> Result<(), BUS::Error> {
        let [[xh, xl], [yh, yl], [zh, zl]]: [[u8; 2]; 3] =
            offsets.map(|x| (-x).to_be_bytes()).into();

        self.set_user_bank(&Bank2::XgOffsUsrh, false).await?;

        self.bus.bus_write(&[Bank1::XaOffsH.reg(), xh, xl]).await?;
        self.bus.bus_write(&[Bank1::YaOffsH.reg(), yh, yl]).await?;
        self.bus.bus_write(&[Bank1::ZaOffsH.reg(), zh, zl]).await?;

        Ok(())
    }

    /// Returns the number of new readings in FIFO buffer
    pub async fn new_data_ready(&mut self) -> u8 {
        self.read_from::<1, _>(Bank0::DataRdyStatus)
            .await
            .map_or(0, |[b]| b & 0b1111)
    }

    /// Returns the scalar corresponding to the unit and range configured
    fn acc_scalar(&self) -> f32 {
        self.config.acc_unit.scalar() / self.config.acc_range.divisor()
    }

    /// Returns the scalar corresponding to the unit and range configured
    fn gyr_scalar(&self) -> f32 {
        self.config.gyr_unit.scalar() / self.config.gyr_range.divisor()
    }
}

/// Collects 6 bytes into a vector of i16 values (acc/gyr only)
fn collect_3xi16(values: [u8; 6]) -> [i16; 3] {
    let [xh, xl, yh, yl, zh, zl] = values;
    [
        i16::from_be_bytes([xh, xl]),
        i16::from_be_bytes([yh, yl]),
        i16::from_be_bytes([zh, zl]),
    ]
}


/// Collects 6 bytes into a vector of i16 values (mag only)
fn collect_3xi16_mag(values: [u8; 6]) -> [i16; 3] {
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

/// Runs any future supplied in this driver after a rising or falling
/// edge of a pin, usually the ICM20948's built-in interrupt.
pub trait WithInterrupt<T, E> {
    fn rising(self, pin: &mut impl Wait) -> impl Future<Output = Result<T, IcmError<E>>>;
    fn falling(self, pin: &mut impl Wait) -> impl Future<Output = Result<T, IcmError<E>>>;
}

impl <T, E, F> WithInterrupt<T, E> for F
where
    E: Into<IcmError<E>>,
    F: Future<Output = Result<T, E>>
{
    async fn rising(self, pin: &mut impl Wait) -> Result<T, IcmError<E>> {
        pin.wait_for_rising_edge().await.map_err(|_|IcmError::InterruptPinError)?;
        Ok(self.await?)
    }

    async fn falling(self, pin: &mut impl Wait) -> Result<T, IcmError<E>> {
        pin.wait_for_falling_edge().await.map_err(|_|IcmError::InterruptPinError)?;
        Ok(self.await?)
    }
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Icm20948Config {
    pub acc_range: AccRange,
    pub gyr_range: GyrRange,
    pub acc_unit: AccUnit,
    pub gyr_unit: GyrUnit,
    pub acc_dlp: AccDlp,
    pub gyr_dlp: GyrDlp,
    pub acc_odr: u16,
    pub gyr_odr: u8,
    pub int: Option<Interrupt>,
}

impl Default for Icm20948Config {
    fn default() -> Self {
        Self {
            acc_range: AccRange::Gs2,
            gyr_range: GyrRange::Dps1000,
            acc_unit: AccUnit::Gs,
            gyr_unit: GyrUnit::Dps,
            acc_dlp: AccDlp::Disabled,
            gyr_dlp: GyrDlp::Disabled,
            acc_odr: 0,
            gyr_odr: 0,
            int: None,
        }
    }
}

#[non_exhaustive]
#[derive(Copy, Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Interrupt {
    // Interrupt pin active low
    pub active_low: bool,

    // Open-drain or push-pull
    pub open_drain: bool,

    // Latch interrupt until cleared
    pub latch_on: bool,

    // Clears interrupt status on any read operation
    pub clear_on_read: bool,

    /// Enable wake on motion interrupt 
    pub wake_on_motion: bool,

    /// Enable PLL ready interrupt
    pub pll_ready: bool,

    /// Enable DMP interrupt
    pub dmp_ready: bool,

    /// Enable I2C master interrupt
    pub i2c_master: bool,

    /// Enable raw data interrupt
    pub raw_data_ready: bool,
}

#[derive(Copy, Clone, Debug)]
pub enum I2cAddress {
    /// On some modules `0x68` is the default address if pin `AD0` is low
    X68,
    /// On some modules `0x69` is the default address if pin `AD0` is high
    X69,
    /// In case the ICM modules has a different address
    Any(u8),
}

impl From<u8> for I2cAddress {
    fn from(address: u8) -> Self {
        I2cAddress::Any(address)
    }
}

impl I2cAddress {
    fn get(&self) -> u8 {
        match self {
            I2cAddress::X68 => 0x68,
            I2cAddress::X69 => 0x69,
            I2cAddress::Any(a) => *a,
        }
    }
}

impl Default for I2cAddress {
    fn default() -> Self {
        I2cAddress::X69
    }
}

/// Range / sensitivity of accelerometer in Gs
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum AccRange {
    Gs2 = 0b00,
    Gs4 = 0b01,
    Gs8 = 0b10,
    Gs16 = 0b11,
}

impl AccRange {
    pub fn divisor(self) -> f32 {
        match self {
            Self::Gs2 => 16384.0,
            Self::Gs4 => 8192.0,
            Self::Gs8 => 4096.0,
            Self::Gs16 => 2048.0,
        }
    }
}

/// Range / sentivity of gyroscope in degrees/second
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GyrRange {
    Dps250 = 0b00,
    Dps500 = 0b01,
    Dps1000 = 0b10,
    Dps2000 = 0b11,
}

impl GyrRange {
    pub fn divisor(self) -> f32 {
        match self {
            Self::Dps250 => 131.0,
            Self::Dps500 => 65.5,
            Self::Dps1000 => 32.8,
            Self::Dps2000 => 16.4,
        }
    }
}

/// Unit of accelerometer readings
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum AccUnit {
    /// Meters per second squared (m/s^2)
    Mpss,
    /// Number of times of normal gravity
    Gs,
}

impl AccUnit {
    pub fn scalar(self) -> f32 {
        match self {
            Self::Mpss => 9.82,
            Self::Gs => 1.0,
        }
    }
}

/// Unit of gyroscope readings
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GyrUnit {
    /// Radians per second
    Rps,
    /// Degrees per second
    Dps,
}

impl GyrUnit {
    pub fn scalar(self) -> f32 {
        match self {
            Self::Rps => 0.017453293,
            Self::Dps => 1.0,
        }
    }
}

/// Digital low-pass filter for of accelerometer readings
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum AccDlp {
    Hz473 = 7,
    Hz246 = 1,
    Hz111 = 2,
    Hz50 = 3,
    Hz24 = 4,
    Hz12 = 5,
    Hz6 = 6,
    Disabled = 8,
}

/// Digital low-pass filter for of gyroscope readings
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GyrDlp {
    Hz361 = 7,
    Hz196 = 0,
    Hz152 = 1,
    Hz120 = 2,
    Hz51 = 3,
    Hz24 = 4,
    Hz12 = 5,
    Hz6 = 6,
    Disabled = 8,
}

#[derive(Debug)]
pub enum IcmError<E> {
    BusError(E),
    ImuSetupError,
    MagSetupError,
    InterruptPinError,
}

impl<E> From<E> for IcmError<E> {
    fn from(error: E) -> Self {
        IcmError::BusError(error)
    }
}
