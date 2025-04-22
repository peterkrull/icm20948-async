#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Config {
    pub acc_range: AccRange,
    pub gyr_range: GyrRange,
    pub acc_unit: AccUnit,
    pub gyr_unit: GyrUnit,
    pub acc_dlp: AccDlp,
    pub gyr_dlp: GyrDlp,
    pub acc_odr: u16,
    pub gyr_odr: u8,
}

impl Default for Config {
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
        }
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub enum I2cAddress {
    /// On some modules `0x68` is the default address if pin `AD0` is low
    X68,
    /// On some modules `0x69` is the default address if pin `AD0` is high
    #[default]
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
    pub const fn get(&self) -> u8 {
        match self {
            I2cAddress::X68 => 0x68,
            I2cAddress::X69 => 0x69,
            I2cAddress::Any(a) => *a,
        }
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
    pub const fn divisor(self) -> f32 {
        match self {
            Self::Gs2 => i16::MAX as f32 / 2.0,
            Self::Gs4 => i16::MAX as f32 / 4.0,
            Self::Gs8 => i16::MAX as f32 / 8.0,
            Self::Gs16 => i16::MAX as f32 / 16.0,
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
    pub const fn divisor(self) -> f32 {
        match self {
            Self::Dps250 => i16::MAX as f32 / 250.,
            Self::Dps500 => i16::MAX as f32 / 500.,
            Self::Dps1000 => i16::MAX as f32 / 1000.,
            Self::Dps2000 => i16::MAX as f32 / 2000.,
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
    pub const fn scalar(self) -> f32 {
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
    pub const fn scalar(self) -> f32 {
        match self {
            Self::Rps => 1.0f32.to_radians(),
            Self::Dps => 1.0f32,
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
