use crate::data_logger::LoggingConfig;
use crate::engine::EngineConfig;
use crate::safety::SafetyConfig;
use crate::telemetry::TelemetryConfig;
use serde::{Deserialize, Serialize};

#[derive(Clone, Serialize, Deserialize)]
pub struct SystemConfig {
    pub vehicle: VehicleConfig,
    pub engine: EngineConfig,
    pub telemetry: TelemetryConfig,
    pub logging: LoggingConfig,
    pub safety: SafetyConfig,
    pub recovery: RecoveryConfig,
    pub calibration: CalibrationConfig,
}

#[derive(Clone, Serialize, Deserialize)]
pub struct VehicleConfig {
    pub name: heapless::String<32>,
    pub mass_kg: f32,
    pub length_m: f32,
    pub diameter_m: f32,
    pub drag_coefficient: f32,
    pub reference_area_m2: f32,
}

#[derive(Clone, Serialize, Deserialize)]
pub struct RecoveryConfig {
    pub drogue_deploy_altitude: f32,
    pub main_deploy_altitude: f32,
    pub drogue_area_m2: f32,
    pub main_area_m2: f32,
    pub deployment_delay_ms: u32,
}

#[derive(Clone, Serialize, Deserialize)]
pub struct CalibrationConfig {
    pub accel_offset: [f32; 3],
    pub accel_scale: [f32; 3],
    pub gyro_offset: [f32; 3],
    pub mag_offset: [f32; 3],
    pub mag_scale: [f32; 3],
    pub baro_offset: f32,
}

impl Default for SystemConfig {
    fn default() -> Self {
        Self {
            vehicle: VehicleConfig {
                name: heapless::String::from("Avion2"),
                mass_kg: 500.0,
                length_m: 10.0,
                diameter_m: 0.5,
                drag_coefficient: 0.75,
                reference_area_m2: 0.196,
            },
            engine: EngineConfig::default(),
            telemetry: TelemetryConfig::default(),
            logging: LoggingConfig::default(),
            safety: SafetyConfig::default(),
            recovery: RecoveryConfig {
                drogue_deploy_altitude: 1000.0,
                main_deploy_altitude: 300.0,
                drogue_area_m2: 1.0,
                main_area_m2: 10.0,
                deployment_delay_ms: 1000,
            },
            calibration: CalibrationConfig {
                accel_offset: [0.0, 0.0, 0.0],
                accel_scale: [1.0, 1.0, 1.0],
                gyro_offset: [0.0, 0.0, 0.0],
                mag_offset: [0.0, 0.0, 0.0],
                mag_scale: [1.0, 1.0, 1.0],
                baro_offset: 0.0,
            },
        }
    }
}

impl SystemConfig {
    pub fn load_from_flash() -> Option<Self> {
        defmt::info!("Loading configuration from flash");
        None
    }

    pub fn save_to_flash(&self) -> bool {
        defmt::info!("Saving configuration to flash");
        true
    }

    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.vehicle.mass_kg <= 0.0 {
            return Err(ConfigError::InvalidVehicleMass);
        }

        if self.recovery.main_deploy_altitude >= self.recovery.drogue_deploy_altitude {
            return Err(ConfigError::InvalidRecoveryAltitudes);
        }

        if self.safety.max_altitude_m <= 0.0 {
            return Err(ConfigError::InvalidSafetyLimits);
        }

        Ok(())
    }

    pub fn apply_calibration(&mut self, cal: &CalibrationConfig) {
        self.calibration = cal.clone();
        defmt::info!("Applied new calibration values");
    }
}

#[derive(Debug)]
pub enum ConfigError {
    InvalidVehicleMass,
    InvalidRecoveryAltitudes,
    InvalidSafetyLimits,
    FlashReadError,
    FlashWriteError,
}

pub struct ConfigManager {
    current_config: SystemConfig,
    backup_config: Option<SystemConfig>,
    config_version: u32,
}

impl ConfigManager {
    pub fn new() -> Self {
        let config = SystemConfig::load_from_flash().unwrap_or_default();

        Self {
            current_config: config.clone(),
            backup_config: Some(config),
            config_version: 1,
        }
    }

    pub fn get_config(&self) -> &SystemConfig {
        &self.current_config
    }

    pub fn update_config(&mut self, new_config: SystemConfig) -> Result<(), ConfigError> {
        new_config.validate()?;

        self.backup_config = Some(self.current_config.clone());
        self.current_config = new_config;
        self.config_version += 1;

        if self.current_config.save_to_flash() {
            defmt::info!("Configuration updated to version {}", self.config_version);
            Ok(())
        } else {
            self.rollback();
            Err(ConfigError::FlashWriteError)
        }
    }

    pub fn rollback(&mut self) {
        if let Some(backup) = &self.backup_config {
            self.current_config = backup.clone();
            defmt::warn!("Configuration rolled back");
        }
    }

    pub fn perform_calibration(&mut self) -> CalibrationResult {
        defmt::info!("Starting calibration sequence");

        let mut result = CalibrationResult::default();

        result.accel_calibrated = self.calibrate_accelerometer();
        result.gyro_calibrated = self.calibrate_gyroscope();
        result.mag_calibrated = self.calibrate_magnetometer();
        result.baro_calibrated = self.calibrate_barometer();

        if result.all_successful() {
            defmt::info!("Calibration completed successfully");
        } else {
            defmt::warn!("Calibration partially failed");
        }

        result
    }

    fn calibrate_accelerometer(&mut self) -> bool {
        defmt::info!("Calibrating accelerometer");
        true
    }

    fn calibrate_gyroscope(&mut self) -> bool {
        defmt::info!("Calibrating gyroscope");
        true
    }

    fn calibrate_magnetometer(&mut self) -> bool {
        defmt::info!("Calibrating magnetometer");
        true
    }

    fn calibrate_barometer(&mut self) -> bool {
        defmt::info!("Calibrating barometer");
        true
    }
}

#[derive(Default)]
pub struct CalibrationResult {
    pub accel_calibrated: bool,
    pub gyro_calibrated: bool,
    pub mag_calibrated: bool,
    pub baro_calibrated: bool,
}

impl CalibrationResult {
    pub fn all_successful(&self) -> bool {
        self.accel_calibrated && self.gyro_calibrated && self.mag_calibrated && self.baro_calibrated
    }
}