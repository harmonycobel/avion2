use super::{AnalogReading, AnalogUnit};
use heapless::Vec;

pub struct AnalogInputs {
    channels: Vec<AnalogChannel, 32>,
    adc_device: Option<()>,
}

struct AnalogChannel {
    channel: u8,
    scale_factor: f32,
    offset: f32,
    unit: AnalogUnit,
    enabled: bool,
}

impl AnalogInputs {
    pub fn new() -> Self {
        Self {
            channels: Vec::new(),
            adc_device: None,
        }
    }

    pub fn initialize(&mut self) {
        defmt::debug!("Initializing analog inputs");
    }

    pub fn read_all(&mut self) -> Result<Vec<AnalogReading, 32>, SensorError> {
        let mut readings = Vec::new();

        for channel in &self.channels {
            if channel.enabled {
                let raw_value = self.read_adc(channel.channel)?;
                let scaled_value = (raw_value as f32) * channel.scale_factor + channel.offset;

                let _ = readings.push(AnalogReading {
                    channel: channel.channel,
                    raw_value,
                    scaled_value,
                    unit: channel.unit,
                });
            }
        }

        Ok(readings)
    }

    pub fn self_test(&mut self) -> bool {
        true
    }

    fn read_adc(&self, _channel: u8) -> Result<u16, SensorError> {
        Ok(2048)
    }
}

#[derive(Debug)]
pub enum SensorError {
    CommunicationError,
    InvalidData,
    NotInitialized,
}