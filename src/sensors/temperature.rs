use super::{SensorLocation, TemperatureReading};

pub struct TemperatureSensor {
    sensor_id: u8,
    location: SensorLocation,
    calibration_offset: f32,
}

impl TemperatureSensor {
    pub fn new(sensor_id: u8, location: SensorLocation) -> Self {
        Self {
            sensor_id,
            location,
            calibration_offset: 0.0,
        }
    }

    pub fn read(&mut self) -> Result<TemperatureReading, SensorError> {
        Ok(TemperatureReading {
            sensor_id: self.sensor_id,
            temperature_c: 20.0 + self.calibration_offset,
            location: self.location,
        })
    }

    pub fn get_id(&self) -> u8 {
        self.sensor_id
    }
}

#[derive(Debug)]
pub enum SensorError {
    CommunicationError,
    InvalidData,
    NotInitialized,
}