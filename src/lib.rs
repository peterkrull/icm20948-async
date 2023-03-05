#![feature(type_changing_struct_update)]
#![no_std]

use core::marker::PhantomData;

use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;
use nalgebra::Vector3;

mod regs;
use regs::*;

const ICM20948_ADDR: u8 = 0x69; // I2C address of the ICM20948
const MAGNET_ADDR: u8 = 0x0C; // I2C address of magnetometer

#[derive(Clone, Copy)]
pub struct Data6Dof {
    pub acc: Vector3<f32>,
    pub gyr: Vector3<f32>,
    pub tmp: f32,
}

#[derive(Clone, Copy)]
pub struct Data9Dof {
    pub acc: Vector3<f32>,
    pub gyr: Vector3<f32>,
    pub mag: Vector3<f32>,
    pub tmp: f32,
}

// Compile-time MAG states
pub struct MagEnabled {
    is_calibrated: bool,
    offset: Vector3<f32>,
    scale: Vector3<f32>,
}
pub struct MagDisabled;

// Compile-time init states
pub struct Init;
pub struct NotInit;

pub struct Icm20948<I2C, MAG, INIT> {
    i2c: I2C,
    addr: u8,
    config: Icm20948Config,
    user_bank: UserBank,
    mag_state: MAG,
    gyr_cal: Vector3<f32>,
    init_state: PhantomData<INIT>,
}

impl<I2C, E> Icm20948<I2C, MagDisabled, NotInit>
where
    I2C: I2c<Error = E>,
    E: Into<IcmError<E>>,
{
    /// Creates an uninitialized IMU struct with the given config.
    pub fn new_from_cfg(i2c: I2C, cfg: Icm20948Config) -> Icm20948<I2C, MagDisabled, NotInit> {
        Self {
            i2c,
            addr: ICM20948_ADDR,
            config: cfg,
            user_bank: UserBank::Bank0,
            mag_state: MagDisabled,
            gyr_cal: Vector3::from_element(0.),
            init_state: PhantomData::<NotInit>,
        }
    }

    /// Creates an uninitialized IMU struct with a default config on address 0x68
    pub fn new(i2c: I2C) -> Icm20948<I2C, MagDisabled, NotInit> {
        Self::new_from_cfg(i2c, Icm20948Config::default())
    }

    /*
        Configuration methods
    */

    pub fn set_address(self, addr: u8) -> Icm20948<I2C, MagDisabled, NotInit> {
        Icm20948 { addr, ..self }
    }

    pub fn acc_range(self, acc_range: AccelerometerRange) -> Icm20948<I2C, MagDisabled, NotInit> {
        Icm20948 { config: Icm20948Config { acc_range, ..self.config }, ..self }
    }

    pub fn acc_dlp(self, acc_dlp: AccelerometerDlp) -> Icm20948<I2C, MagDisabled, NotInit> {
        Icm20948 { config: Icm20948Config { acc_dlp: Some(acc_dlp), ..self.config },..self }
    }

    pub fn acc_unit(self, acc_unit: AccelerometerUnit) -> Icm20948<I2C, MagDisabled, NotInit> {
        Icm20948 { config: Icm20948Config { acc_unit, ..self.config }, ..self }
    }
    pub fn gyro_range(self, gyro_range: GyroscopeRange) -> Icm20948<I2C, MagDisabled, NotInit> {
        Icm20948 { config: Icm20948Config { gyro_range, ..self.config }, ..self }
    }

    pub fn gyro_dlp(self, gyro_dlp: GyroscopeDlp) -> Icm20948<I2C, MagDisabled, NotInit> {
        Icm20948 { config: Icm20948Config { gyro_dlp: Some(gyro_dlp), ..self.config }, ..self }
    }

    pub fn gyro_unit(self, gyro_unit: GyroscopeUnit) -> Icm20948<I2C, MagDisabled, NotInit> {
        Icm20948 { config: Icm20948Config { gyro_unit, ..self.config }, ..self }
    }

    /*
        Initialization methods
    */

    pub async fn initialize_6dof(
        mut self,
    ) -> Result<Icm20948<I2C, MagDisabled, Init>, IcmError<E>> {
        self.setup_acc_gyr().await?;

        Ok(Icm20948 {
            mag_state: MagDisabled,
            init_state: PhantomData::<Init>,
            ..self
        })
    }

    pub async fn initialize_9dof(mut self) -> Result<Icm20948<I2C, MagEnabled, Init>, IcmError<E>> {
        self.setup_acc_gyr().await?;
        self.setup_mag().await?;

        Ok(Icm20948 {
            mag_state: MagEnabled {
                is_calibrated: false,
                offset: Vector3::from_element(1.),
                scale: Vector3::from_element(1.),
            },
            init_state: PhantomData::<Init>,
            ..self
        })
    }

    /// Setup accelerometer and gyroscope according to config
    async fn setup_acc_gyr(&mut self) -> Result<(), IcmError<E>> {
        // Ensure known-good state
        self.device_reset().await?;

        // Initially set user bank by force, and check identity
        self.set_user_bank(&Bank0::WhoAmI, true).await?;
        let [id] = self.read_from(Bank0::WhoAmI).await?;

        if id != 0xEA {
            return Err(IcmError::ImuSetupError);
        }

        // Disable sleep mode and set to auto select clock source
        self.write_to(Bank0::PwrMgmt1, 0x01).await?;

        // Set gyro and accel ranges
        self.set_acc_range(self.config.acc_range).await?;
        self.set_gyr_range(self.config.gyro_range).await?;

        // Apply digital lowpass filter settings
        self.set_gyr_dlp(self.config.gyro_dlp).await?;
        self.set_acc_dlp(self.config.acc_dlp).await?;

        Ok(())
    }

    /// Setup magnetometer in continuous mode
    async fn setup_mag(&mut self) -> Result<(), IcmError<E>> {

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
        self.write_to(Bank3::I2cSlv0Reg, MagBank::XDataLow.reg()).await?;

        // Set expected read size
        self.write_to(Bank3::I2cSlv0Ctrl, 1 << 7 | 8).await?;

        Ok(())
    }    
}

impl<I2C, E, MAG, INIT> Icm20948<I2C, MAG, INIT>
where
    I2C: I2c<Error = E>,
    E: Into<IcmError<E>>,
{
    pub async fn device_reset(&mut self) -> Result<(), E> {
        Timer::after(Duration::from_millis(20)).await;
        self.write_to_flag(Bank0::PwrMgmt1, 1 << 7, 1 << 7).await?;
        Timer::after(Duration::from_millis(50)).await;
        Ok(())
    }

    async fn enable_i2c_master(&mut self, enable: bool) -> Result<(), E> {
        self.write_to_flag(Bank0::UserCtrl, (enable as u8) << 5, 1 << 5)
            .await
    }

    async fn reset_i2c_master(&mut self) -> Result<(), E> {
        self.write_to_flag(Bank0::UserCtrl, 1 << 1, 1 << 1).await
    }

    /// Ensure correct user bank for given register
    async fn set_user_bank<R: Register>(&mut self, bank: &R, force: bool) -> Result<(), E> {
        if (self.user_bank != bank.bank()) || force {
            self.i2c
                .write(
                    self.addr,
                    &[regs::REG_BANK_SEL, (bank.bank() as u8) << 4],
                )
                .await?;
            self.user_bank = bank.bank();
        }
        Ok(())
    }

    async fn read_from<const N: usize, R: Register>(&mut self, cmd: R) -> Result<[u8; N], E> {
        let mut buf = [0u8; N];
        self.set_user_bank(&cmd, false).await?;
        self.i2c.write_read(self.addr, &[cmd.reg()], &mut buf) .await?;
        Ok(buf)
    }

    async fn write_to<R: Register>(&mut self, cmd: R, data: u8) -> Result<(), E> {
        self.set_user_bank(&cmd, false).await?;
        self.i2c.write(self.addr, &[cmd.reg(), data]).await?;
        Ok(())
    }

    /// Write to a register, but only overwrite the parts corresponding to the flag byte
    async fn write_to_flag<R>(&mut self, cmd: R, data: u8, flag: u8) -> Result<(), E>
    where
        R: Register + Copy + Clone,
    {
        let [mut register] = self.read_from(cmd).await?;
        register = (register & !flag) | (data & flag);
        self.write_to(cmd, register).await?;
        Ok(())
    }

    async fn set_mag_read(&mut self) -> Result<(), E> {
        let [mut reg] = self.read_from(Bank3::I2cSlv0Addr).await?;
        reg &= 0b01111111;
        reg |= 1 << 7;
        self.write_to(Bank3::I2cSlv0Addr, reg).await
    }

    async fn set_mag_write(&mut self) -> Result<(), E> {
        let [mut reg] = self.read_from(Bank3::I2cSlv0Addr).await?;
        reg &= 0b01111111;
        self.write_to(Bank3::I2cSlv0Addr, reg).await
    }

    async fn mag_write_to(&mut self, cmd: u8, data: u8) -> Result<(), E> {
        self.set_mag_write().await?;
        Timer::after(Duration::from_millis(10)).await;
        self.write_to(Bank3::I2cSlv0Reg, cmd).await?;
        self.write_to(Bank3::I2cSlv0Do, data).await?;
        self.write_to(Bank3::I2cSlv0Ctrl, 1 << 7 | 1).await?;
        Timer::after(Duration::from_millis(10)).await;
        self.set_mag_read().await
    }

    async fn mag_read_from<const N: usize>(&mut self, cmd: MagBank) -> Result<[u8; N], E> {
        self.set_mag_read().await?;
        Timer::after(Duration::from_millis(10)).await;
        self.write_to(Bank3::I2cSlv0Reg, cmd.reg()).await?;
        self.write_to(Bank3::I2cSlv0Ctrl, 1 << 7 | N as u8).await?;
        Timer::after(Duration::from_millis(10)).await;
        self.read_from(Bank0::ExtSlvSensData00).await
    }


    async fn mag_reset(&mut self) -> Result<(), E> {
        // Control 3 register bit 1 resets magnetometer unit 
        self.mag_write_to(MagBank::Control3.reg(), 1).await?;
        Timer::after(Duration::from_millis(100)).await;

        // Reset i2c master module
        self.reset_i2c_master().await
    }

    pub async fn set_acc_range(&mut self, range: AccelerometerRange) -> Result<(), E> {
        self.write_to_flag(Bank2::AccelConfig, (range as u8) << 1, 0b0110)
            .await?;
        self.config.acc_range = range;
        Ok(())
    }

    pub async fn set_gyr_range(&mut self, range: GyroscopeRange) -> Result<(), E> {
        self.write_to_flag(Bank2::GyroConfig1, (range as u8) << 1, 0b0110)
            .await?;
        self.config.gyro_range = range;
        Ok(())
    }

    pub fn set_acc_unit(&mut self, unit: AccelerometerUnit) {
        self.config.acc_unit = unit;
    }

    pub fn set_gyr_unit(&mut self, unit: GyroscopeUnit) {
        self.config.gyro_unit = unit;
    }
    
    pub async fn set_acc_dlp(&mut self, acc_dlp: Option<AccelerometerDlp>) -> Result<(), E> {
        if let Some(dlp) = acc_dlp {
            self.write_to_flag(Bank2::AccelConfig, (dlp as u8) << 3 | 1, 0b111001).await
        } else {
            self.write_to_flag(Bank2::AccelConfig, 0u8, 0b111001).await
        }
    }

    pub async fn set_gyr_dlp(&mut self, gyr_dlp: Option<GyroscopeDlp>) -> Result<(), E> {
        if let Some(dlp) = gyr_dlp {
            self.write_to_flag(Bank2::GyroConfig1, (dlp as u8) << 3 | 1, 0b111001).await
        } else {
            self.write_to_flag(Bank2::GyroConfig1, 0u8, 0b111001).await
        }
    }
}

impl<I2C, E> Icm20948<I2C, MagEnabled, Init>
where
    I2C: I2c<Error = E>,
    E: Into<IcmError<E>>,
{
    fn apply_mag_calibration(&self, mag: & mut Vector3<f32>) {
        if self.mag_state.is_calibrated {
            *mag = mag.zip_zip_map(&self.mag_state.offset,&self.mag_state.scale, |m,o,s| {
                ( m - o ) / s
            });
        }
    }

    pub fn set_mag_calibration(&mut self, offset: [f32; 3], scale: [f32; 3]) {
        self.mag_state.is_calibrated = true;
        self.mag_state.offset = offset.into();
        self.mag_state.scale = scale.into();
    }

    pub fn reset_mag_calibration(&mut self) {
        self.mag_state.is_calibrated = false;
        self.mag_state.offset = Vector3::from_element(1.);
        self.mag_state.scale = Vector3::from_element(1.);
    }

    /// Get vector of scaled magnetometer values
    pub async fn read_mag(&mut self) -> Result<Vector3<f32>, E> {
        let mut mag = self.read_mag_unscaled().await?
        .map(|x| (x as f32)).into();
        self.apply_mag_calibration(&mut mag);

        Ok(mag)
    }

    /// Get array of unscaled accelerometer values
    pub async fn read_mag_unscaled(&mut self) -> Result<[i16; 3], E> {
        let raw: [u8; 6] = self.read_from(Bank0::ExtSlvSensData00).await?;
        let mag = collect_3xi16_mag(raw);

        Ok(mag)
    }

    /// Get scaled measurement for accelerometer, gyroscope and magnetometer, and temperature
    pub async fn read_all(&mut self) -> Result<Data9Dof, E> {
        let raw: [u8; 20] = self.read_from(Bank0::AccelXoutH).await?;
        let [axh, axl, ayh, ayl, azh, azl, gxh, gxl, gyh, gyl, gzh, gzl, tph, tpl, mxl, mxh, myl, myh, mzl, mzh] =
            raw;

        let acc = self.scaled_acc_from_bytes([axh, axl, ayh, ayl, azh, azl]);
        let gyr = self.scaled_gyr_from_bytes([gxh, gxl, gyh, gyl, gzh, gzl]);
        let mag = self.scaled_mag_from_bytes([mxl, mxh, myl, myh, mzl, mzh]);

        let tmp = self.scaled_tmp_from_bytes([tph, tpl]);

        Ok(Data9Dof { acc, gyr, mag, tmp })
    }

    fn scaled_mag_from_bytes(&self, bytes: [u8; 6]) -> Vector3<f32> {
        let mut mag = collect_3xi16_mag(bytes)
        .map(|x| (x as f32)).into();
        self.apply_mag_calibration(&mut mag);
        mag
    }
}

impl<I2C, E> Icm20948<I2C, MagDisabled, Init>
where
    I2C: I2c<Error = E>,
    E: Into<IcmError<E>>,
{
    /// Get scaled measurements for accelerometer and gyroscope, and temperature
    pub async fn read_all(&mut self) -> Result<Data6Dof, E> {
        let raw: [u8; 14] = self.read_from(Bank0::AccelXoutH).await?;
        let [axh, axl, ayh, ayl, azh, azl, gxh, gxl, gyh, gyl, gzh, gzl, tph, tpl] = raw;

        let acc = self.scaled_acc_from_bytes([axh, axl, ayh, ayl, azh, azl]);
        let gyr = self.scaled_gyr_from_bytes([gxh, gxl, gyh, gyl, gzh, gzl]);

        let tmp = self.scaled_tmp_from_bytes([tph, tpl]);

        Ok(Data6Dof { acc, gyr, tmp })
    }
}

impl<I2C, E, MAG> Icm20948<I2C, MAG, Init>
where
    I2C: I2c<Error = E>,
    E: Into<IcmError<E>>,
{

    fn scaled_acc_from_bytes(&self, bytes: [u8; 6]) -> Vector3<f32> {
        let acc = collect_3xi16(bytes).map(|x| (x as f32) * self.acc_scalar());
        Vector3::from(acc)
    }

    fn scaled_gyr_from_bytes(&self, bytes: [u8; 6]) -> Vector3<f32> {
        let gyr = collect_3xi16(bytes).map(|x| (x as f32) * self.gyr_scalar());
        Vector3::from(gyr) - self.gyr_cal
    }

    fn scaled_tmp_from_bytes(&self, bytes: [u8; 2]) -> f32 {
        i16::from_be_bytes(bytes) as f32 / 333.87 - 21.
    }

    pub fn set_gyr_offsets(&mut self, offsets: Vector3<f32>) {
        self.gyr_cal = offsets;
    }

    /// Get array of unscaled accelerometer values
    pub async fn read_acc_unscaled(&mut self) -> Result<Vector3<i16>, E> {
        let raw = self.read_from(Bank0::AccelXoutH).await?;
        let [axh, axl, ayh, ayl, azh, azl] = raw;
        Ok(collect_3xi16([axh, axl, ayh, ayl, azh, azl]).into())
    }

    /// Get array of scaled accelerometer values
    pub async fn read_acc(&mut self) -> Result<Vector3<f32>, E> {
        let acc = self
            .read_acc_unscaled()
            .await?
            .map(|x| (x as f32) * self.acc_scalar());
        Ok(acc)
    }

    /// Get array of unscaled gyroscope values
    pub async fn read_gyr_unscaled(&mut self) -> Result<Vector3<i16>, E> {
        let raw = self.read_from(Bank0::GyroXoutH).await?;
        let [gxh, gxl, gyh, gyl, gzh, gzl] = raw;
        Ok(collect_3xi16([gxh, gxl, gyh, gyl, gzh, gzl]).into())
    }

    /// Get array of scaled gyroscope values
    pub async fn read_gyr(&mut self) -> Result<Vector3<f32>, E> {
        let gyr = self
            .read_gyr_unscaled()
            .await?
            .map(|x| (x as f32) * self.gyr_scalar());
        Ok(gyr - self.gyr_cal)
    }

    #[inline(always)]
    fn acc_scalar(&self) -> f32 {
        self.config.acc_unit.scalar() / self.config.acc_range.divisor()
    }

    #[inline(always)]
    fn gyr_scalar(&self) -> f32 {
        self.config.gyro_unit.scalar() / self.config.gyro_range.divisor()
    }

}

fn collect_3xi16(values: [u8; 6]) -> [i16; 3] {
    let [xh, xl, yh, yl, zh, zl] = values;
    [
        i16::from_be_bytes([xh, xl]),
        i16::from_be_bytes([yh, yl]),
        i16::from_be_bytes([zh, zl]),
    ]
}

fn collect_3xi16_mag(values: [u8; 6]) -> [i16; 3] {
    let [xl, xh, yl, yh, zl, zh] = values;
    [
        i16::from_be_bytes([xh, xl]),
        -i16::from_be_bytes([yh, yl]),
        -i16::from_be_bytes([zh, zl]),
    ]
}

// Internal enums and structs

#[derive(Copy, Clone)]
pub struct Icm20948Config {
    acc_range: AccelerometerRange,
    gyro_range: GyroscopeRange,
    acc_unit: AccelerometerUnit,
    gyro_unit: GyroscopeUnit,
    acc_dlp: Option<AccelerometerDlp>,
    gyro_dlp: Option<GyroscopeDlp>,
}

impl Default for Icm20948Config {
    fn default() -> Self {
        Self {
            acc_range: AccelerometerRange::Gs2,
            gyro_range: GyroscopeRange::Dps1000,
            acc_unit: AccelerometerUnit::Gs,
            gyro_unit: GyroscopeUnit::Dps,
            acc_dlp: None,
            gyro_dlp: None,
        }
    }
}

#[derive(Copy, Clone)]
pub enum AccelerometerRange {
    Gs2 = 0b00,
    Gs4 = 0b01,
    Gs8 = 0b10,
    Gs16 = 0b11,
}

impl AccelerometerRange {
    fn divisor(&self) -> f32 {
        match self {
            Self::Gs2 => 16384.0,
            Self::Gs4 => 8192.0,
            Self::Gs8 => 4096.0,
            Self::Gs16 => 2048.0,
        }
    }
}

#[derive(Copy, Clone)]
pub enum GyroscopeRange {
    Dps250 = 0b00,
    Dps500 = 0b01,
    Dps1000 = 0b10,
    Dps2000 = 0b11,
}

impl GyroscopeRange {
    fn divisor(&self) -> f32 {
        match self {
            Self::Dps250 => 131.0,
            Self::Dps500 => 65.5,
            Self::Dps1000 => 32.8,
            Self::Dps2000 => 16.4,
        }
    }
}

#[derive(Copy, Clone)]
pub enum AccelerometerUnit {
    /// Meters per second squared (m/s^2)
    Mpss,
    /// Number of times of normal gravity
    Gs,
}

impl AccelerometerUnit {
    fn scalar(&self) -> f32 {
        match self {
            Self::Mpss => 9.82,
            Self::Gs => 1.0,
        }
    }
}

#[derive(Copy, Clone)]
pub enum GyroscopeUnit {
    /// Radians per second
    Rps,
    /// Degrees per second
    Dps,
}

impl GyroscopeUnit {
    fn scalar(&self) -> f32 {
        match self {
            Self::Rps => 0.017453293,
            Self::Dps => 1.0,
        }
    }
}

#[derive(Copy, Clone)]
pub enum AccelerometerDlp {
    Hz473 = 7,
    Hz246 = 1,
    Hz111 = 2,
    Hz50 = 3,
    Hz24 = 4,
    Hz12 = 5,
    Hz6 = 6,
}

#[derive(Copy, Clone)]
pub enum GyroscopeDlp {
    Hz361 = 7,
    Hz196 = 0,
    Hz152 = 1,
    Hz120 = 2,
    Hz51 = 3,
    Hz24 = 4,
    Hz12 = 5,
    Hz6 = 6,
}

#[derive(Debug)]
pub enum IcmError<E> {
    BusError(E),
    ImuSetupError,
    MagSetupError,
}

impl<E> From<E> for IcmError<E> {
    fn from(error: E) -> Self {
        IcmError::BusError(error)
    }
}
