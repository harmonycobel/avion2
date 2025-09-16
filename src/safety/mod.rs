use crate::sensors::SensorData;
use heapless::Vec;

#[derive(Clone)]
pub struct SafetyConfig {
    pub max_altitude_m: f32,
    pub max_velocity_mps: f32,
    pub max_acceleration_g: f32,
    pub max_rotation_rate_dps: f32,
    pub geofence_radius_m: f32,
    pub min_battery_voltage: f32,
    pub max_temperature_c: f32,
    pub max_chamber_pressure_pa: f32,
    pub abort_on_loss_of_telemetry: bool,
    pub abort_timeout_ms: u32,
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            max_altitude_m: 10000.0,
            max_velocity_mps: 500.0,
            max_acceleration_g: 15.0,
            max_rotation_rate_dps: 360.0,
            geofence_radius_m: 5000.0,
            min_battery_voltage: 10.0,
            max_temperature_c: 85.0,
            max_chamber_pressure_pa: 8000000.0,
            abort_on_loss_of_telemetry: true,
            abort_timeout_ms: 5000,
        }
    }
}

pub struct SafetyMonitor {
    config: SafetyConfig,
    abort_requested: bool,
    violations: Vec<SafetyViolation, 16>,
    last_telemetry_ms: u32,
    flight_termination_system: FlightTerminationSystem,
    range_safety: RangeSafety,
}

#[derive(Debug, Clone)]
pub enum SafetyViolation {
    AltitudeExceeded { altitude: f32 },
    VelocityExceeded { velocity: f32 },
    AccelerationExceeded { acceleration: f32 },
    RotationRateExceeded { rate: f32 },
    GeofenceBreach { distance: f32 },
    LowBattery { voltage: f32 },
    OverTemperature { temperature: f32, location: String },
    OverPressure { pressure: f32, location: String },
    TelemetryLost { duration_ms: u32 },
    SensorFailure { sensor: String },
    StructuralAnomaly,
}

struct FlightTerminationSystem {
    armed: bool,
    triggered: bool,
    pyro_channels: Vec<PyroChannel, 4>,
}

struct PyroChannel {
    id: u8,
    armed: bool,
    fired: bool,
    continuity_ok: bool,
}

struct RangeSafety {
    enabled: bool,
    launch_site_lat: f64,
    launch_site_lon: f64,
    max_range_m: f32,
    impact_prediction_enabled: bool,
}

impl SafetyMonitor {
    pub fn new(config: SafetyConfig) -> Self {
        let mut pyro_channels = Vec::new();
        for i in 0..4 {
            let _ = pyro_channels.push(PyroChannel {
                id: i,
                armed: false,
                fired: false,
                continuity_ok: true,
            });
        }

        Self {
            config,
            abort_requested: false,
            violations: Vec::new(),
            last_telemetry_ms: 0,
            flight_termination_system: FlightTerminationSystem {
                armed: false,
                triggered: false,
                pyro_channels,
            },
            range_safety: RangeSafety {
                enabled: true,
                launch_site_lat: 0.0,
                launch_site_lon: 0.0,
                max_range_m: 5000.0,
                impact_prediction_enabled: true,
            },
        }
    }

    pub fn initialize(&mut self) {
        defmt::info!("Initializing safety monitor");
        self.test_pyro_continuity();
        self.arm_flight_termination_system();
    }

    pub fn check(&mut self, sensor_data: &SensorData) {
        self.violations.clear();

        self.check_altitude(sensor_data);
        self.check_acceleration(sensor_data);
        self.check_rotation_rate(sensor_data);
        self.check_temperatures(sensor_data);
        self.check_pressures(sensor_data);
        self.check_sensor_health(sensor_data);
        self.check_telemetry_timeout();
        self.check_geofence(sensor_data);

        if !self.violations.is_empty() {
            for violation in &self.violations {
                defmt::warn!("Safety violation: {:?}", violation);
            }

            if self.should_abort() {
                self.request_abort();
            }
        }
    }

    pub fn is_abort_required(&self) -> bool {
        self.abort_requested
    }

    pub fn request_abort(&mut self) {
        if !self.abort_requested {
            defmt::error!("ABORT REQUESTED BY SAFETY MONITOR");
            self.abort_requested = true;

            if self.flight_termination_system.armed {
                self.trigger_flight_termination();
            }
        }
    }

    pub fn arm_fts(&mut self) -> bool {
        self.arm_flight_termination_system()
    }

    pub fn get_violations(&self) -> &[SafetyViolation] {
        &self.violations
    }

    fn check_altitude(&mut self, sensor_data: &SensorData) {
        let altitude = sensor_data.barometer.altitude_m;
        if altitude > self.config.max_altitude_m {
            let _ = self.violations.push(SafetyViolation::AltitudeExceeded { altitude });
        }
    }

    fn check_acceleration(&mut self, sensor_data: &SensorData) {
        let accel_g = sensor_data.imu.accel.magnitude() / 9.81;
        if accel_g > self.config.max_acceleration_g {
            let _ = self.violations.push(SafetyViolation::AccelerationExceeded {
                acceleration: accel_g,
            });
        }
    }

    fn check_rotation_rate(&mut self, sensor_data: &SensorData) {
        let gyro_dps = sensor_data.imu.gyro.magnitude() * 180.0 / core::f32::consts::PI;
        if gyro_dps > self.config.max_rotation_rate_dps {
            let _ = self.violations.push(SafetyViolation::RotationRateExceeded { rate: gyro_dps });
        }
    }

    fn check_temperatures(&mut self, sensor_data: &SensorData) {
        for temp in &sensor_data.temperatures {
            if temp.temperature_c > self.config.max_temperature_c {
                let _ = self.violations.push(SafetyViolation::OverTemperature {
                    temperature: temp.temperature_c,
                    location: format!("{:?}", temp.location),
                });
            }
        }
    }

    fn check_pressures(&mut self, sensor_data: &SensorData) {
        for pressure in &sensor_data.pressures {
            if matches!(pressure.location, crate::sensors::SensorLocation::EngineChambered) {
                if pressure.pressure_pa > self.config.max_chamber_pressure_pa {
                    let _ = self.violations.push(SafetyViolation::OverPressure {
                        pressure: pressure.pressure_pa,
                        location: format!("{:?}", pressure.location),
                    });
                }
            }
        }
    }

    fn check_sensor_health(&mut self, sensor_data: &SensorData) {
        if !sensor_data.sensor_status.imu_ok {
            let _ = self.violations.push(SafetyViolation::SensorFailure {
                sensor: "IMU".to_string(),
            });
        }

        if !sensor_data.sensor_status.baro_ok {
            let _ = self.violations.push(SafetyViolation::SensorFailure {
                sensor: "Barometer".to_string(),
            });
        }
    }

    fn check_telemetry_timeout(&mut self) {
        if self.config.abort_on_loss_of_telemetry {
            let current_time = self.get_time_ms();
            let time_since_telemetry = current_time - self.last_telemetry_ms;

            if time_since_telemetry > self.config.abort_timeout_ms {
                let _ = self.violations.push(SafetyViolation::TelemetryLost {
                    duration_ms: time_since_telemetry,
                });
            }
        }
    }

    fn check_geofence(&mut self, sensor_data: &SensorData) {
        if !self.range_safety.enabled || !sensor_data.gps.valid {
            return;
        }

        let distance = self.calculate_distance_from_launch(
            sensor_data.gps.latitude,
            sensor_data.gps.longitude,
        );

        if distance > self.config.geofence_radius_m {
            let _ = self.violations.push(SafetyViolation::GeofenceBreach { distance });
        }
    }

    fn should_abort(&self) -> bool {
        for violation in &self.violations {
            match violation {
                SafetyViolation::AltitudeExceeded { .. } => return true,
                SafetyViolation::GeofenceBreach { .. } => return true,
                SafetyViolation::OverPressure { .. } => return true,
                SafetyViolation::StructuralAnomaly => return true,
                SafetyViolation::TelemetryLost { duration_ms } => {
                    if *duration_ms > self.config.abort_timeout_ms {
                        return true;
                    }
                }
                _ => {}
            }
        }
        false
    }

    fn arm_flight_termination_system(&mut self) -> bool {
        if !self.flight_termination_system.armed {
            defmt::warn!("Arming Flight Termination System");

            for channel in &mut self.flight_termination_system.pyro_channels {
                channel.armed = channel.continuity_ok;
            }

            self.flight_termination_system.armed = true;
        }
        true
    }

    fn trigger_flight_termination(&mut self) {
        if self.flight_termination_system.armed && !self.flight_termination_system.triggered {
            defmt::error!("FLIGHT TERMINATION TRIGGERED");

            for channel in &mut self.flight_termination_system.pyro_channels {
                if channel.armed && !channel.fired {
                    self.fire_pyro_channel(channel.id);
                    channel.fired = true;
                }
            }

            self.flight_termination_system.triggered = true;
        }
    }

    fn fire_pyro_channel(&self, channel_id: u8) {
        defmt::error!("Firing pyro channel {}", channel_id);
    }

    fn test_pyro_continuity(&mut self) {
        for channel in &mut self.flight_termination_system.pyro_channels {
            channel.continuity_ok = true;
        }
    }

    fn calculate_distance_from_launch(&self, lat: f64, lon: f64) -> f32 {
        let r_earth = 6371000.0;
        let lat1 = self.range_safety.launch_site_lat.to_radians();
        let lat2 = lat.to_radians();
        let delta_lat = (lat - self.range_safety.launch_site_lat).to_radians();
        let delta_lon = (lon - self.range_safety.launch_site_lon).to_radians();

        let a = (delta_lat / 2.0).sin().powi(2)
            + lat1.cos() * lat2.cos() * (delta_lon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

        (r_earth * c) as f32
    }

    fn get_time_ms(&self) -> u32 {
        0
    }
}

use core::fmt::Write;
use heapless::String;

fn format<T: core::fmt::Debug>(value: T) -> String<32> {
    let mut s = String::new();
    let _ = write!(s, "{:?}", value);
    s
}