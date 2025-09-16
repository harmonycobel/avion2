use super::BarometerData;

pub struct Barometer {
    i2c_device: Option<()>,
    sea_level_pressure_pa: f32,
    last_altitude: f32,
    last_time_ms: u32,
}

impl Barometer {
    pub fn new() -> Self {
        Self {
            i2c_device: None,
            sea_level_pressure_pa: 101325.0,
            last_altitude: 0.0,
            last_time_ms: 0,
        }
    }

    pub fn initialize(&mut self) {
        defmt::debug!("Initializing barometer");
    }

    pub fn read(&mut self) -> Result<BarometerData, SensorError> {
        let pressure_pa = 101325.0;
        let altitude_m = self.pressure_to_altitude(pressure_pa);

        let current_time_ms = 0;
        let dt = (current_time_ms - self.last_time_ms) as f32 / 1000.0;
        let vertical_velocity_mps = if dt > 0.0 {
            (altitude_m - self.last_altitude) / dt
        } else {
            0.0
        };

        self.last_altitude = altitude_m;
        self.last_time_ms = current_time_ms;

        Ok(BarometerData {
            pressure_pa,
            altitude_m,
            temperature_c: 20.0,
            vertical_velocity_mps,
        })
    }

    pub fn self_test(&mut self) -> bool {
        true
    }

    fn pressure_to_altitude(&self, pressure_pa: f32) -> f32 {
        44330.0 * (1.0 - (pressure_pa / self.sea_level_pressure_pa).powf(1.0 / 5.255))
    }
}

#[derive(Debug)]
pub enum SensorError {
    CommunicationError,
    InvalidData,
    NotInitialized,
}