use super::{PressureReading, SensorLocation};

pub struct PressureSensor {
    sensor_id: u8,
    location: SensorLocation,
    max_pressure_pa: f32,
    calibration_offset: f32,
}

impl PressureSensor {
    pub fn new(sensor_id: u8, location: SensorLocation, max_pressure_pa: f32) -> Self {
        Self {
            sensor_id,
            location,
            max_pressure_pa,
            calibration_offset: 0.0,
        }
    }

    pub fn read(&mut self) -> Result<PressureReading, SensorError> {
        Ok(PressureReading {
            sensor_id: self.sensor_id,
            pressure_pa: 101325.0 + self.calibration_offset,
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