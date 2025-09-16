use super::ImuData;
use nalgebra::Vector3;

pub struct ImuSensor {
    spi_device: Option<()>,
    config: ImuConfig,
}

#[derive(Clone)]
pub struct ImuConfig {
    pub accel_range_g: f32,
    pub gyro_range_dps: f32,
    pub sample_rate_hz: u16,
    pub enable_magnetometer: bool,
}

impl Default for ImuConfig {
    fn default() -> Self {
        Self {
            accel_range_g: 16.0,
            gyro_range_dps: 2000.0,
            sample_rate_hz: 1000,
            enable_magnetometer: true,
        }
    }
}

impl ImuSensor {
    pub fn new() -> Self {
        Self {
            spi_device: None,
            config: ImuConfig::default(),
        }
    }

    pub fn initialize(&mut self) {
        defmt::debug!("Initializing IMU sensor");
    }

    pub fn read(&mut self) -> Result<ImuData, SensorError> {
        Ok(ImuData {
            accel: Vector3::new(0.0, 0.0, -9.81),
            gyro: Vector3::new(0.0, 0.0, 0.0),
            mag: Vector3::new(20.0, 5.0, -40.0),
            temperature: 25.0,
        })
    }

    pub fn self_test(&mut self) -> bool {
        true
    }

    pub fn get_id(&self) -> u8 {
        0
    }
}

#[derive(Debug)]
pub enum SensorError {
    CommunicationError,
    InvalidData,
    NotInitialized,
}