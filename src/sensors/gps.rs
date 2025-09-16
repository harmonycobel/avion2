use super::{GpsData, GpsFixType};
use nalgebra::Vector3;

pub struct GpsReceiver {
    uart_device: Option<()>,
    last_valid_position: Option<GpsData>,
}

impl GpsReceiver {
    pub fn new() -> Self {
        Self {
            uart_device: None,
            last_valid_position: None,
        }
    }

    pub fn initialize(&mut self) {
        defmt::debug!("Initializing GPS receiver");
    }

    pub fn read(&mut self) -> Result<GpsData, SensorError> {
        Ok(GpsData {
            valid: false,
            latitude: 0.0,
            longitude: 0.0,
            altitude_m: 0.0,
            velocity_ned: Vector3::new(0.0, 0.0, 0.0),
            hdop: 99.0,
            num_satellites: 0,
            fix_type: GpsFixType::NoFix,
        })
    }

    pub fn self_test(&mut self) -> bool {
        true
    }
}

#[derive(Debug)]
pub enum SensorError {
    CommunicationError,
    InvalidData,
    NotInitialized,
}