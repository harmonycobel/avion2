use crate::navigation::NavigationSolution;
use crate::sensors::SensorData;
use crate::state_machine::FlightState;
use heapless::Vec;
use postcard;
use serde::{Deserialize, Serialize};

#[derive(Clone)]
pub struct TelemetryConfig {
    pub downlink_rate_hz: u32,
    pub uplink_enabled: bool,
    pub encryption_enabled: bool,
    pub frequency_mhz: f32,
    pub power_dbm: i8,
}

impl Default for TelemetryConfig {
    fn default() -> Self {
        Self {
            downlink_rate_hz: 10,
            uplink_enabled: true,
            encryption_enabled: false,
            frequency_mhz: 433.0,
            power_dbm: 20,
        }
    }
}

pub struct TelemetrySystem {
    config: TelemetryConfig,
    radio_device: Option<()>,
    packet_counter: u32,
    uplink_buffer: Vec<u8, 256>,
    recovery_beacon_enabled: bool,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct TelemetryPacket {
    pub packet_id: u32,
    pub timestamp_ms: u32,
    pub flight_state: u8,
    pub position: PositionData,
    pub velocity: VelocityData,
    pub attitude: AttitudeData,
    pub sensors: CompressedSensorData,
    pub system_health: SystemHealth,
    pub crc: u16,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct PositionData {
    pub latitude: i32,
    pub longitude: i32,
    pub altitude_m: i16,
    pub gps_valid: bool,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct VelocityData {
    pub velocity_north: i16,
    pub velocity_east: i16,
    pub velocity_down: i16,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct AttitudeData {
    pub roll_deg: i16,
    pub pitch_deg: i16,
    pub yaw_deg: i16,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CompressedSensorData {
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
    pub baro_altitude: i16,
    pub temperature: i8,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SystemHealth {
    pub battery_voltage: u8,
    pub battery_current: u8,
    pub cpu_usage: u8,
    pub memory_free: u16,
    pub sensor_status: u8,
}

#[derive(Serialize, Deserialize, Debug)]
pub enum UplinkCommand {
    Arm,
    Disarm,
    Ignite,
    Abort,
    SetParameter { param_id: u16, value: f32 },
    RequestStatus,
    EnableRecoveryBeacon,
    ResetSystem,
}

impl TelemetrySystem {
    pub fn new(config: TelemetryConfig) -> Self {
        Self {
            config,
            radio_device: None,
            packet_counter: 0,
            uplink_buffer: Vec::new(),
            recovery_beacon_enabled: false,
        }
    }

    pub fn initialize(&mut self) {
        defmt::info!("Initializing telemetry system");
        defmt::info!(
            "Frequency: {} MHz, Power: {} dBm",
            self.config.frequency_mhz,
            self.config.power_dbm
        );
    }

    pub fn send_telemetry(
        &mut self,
        sensor_data: &SensorData,
        nav_solution: &NavigationSolution,
        flight_state: FlightState,
    ) {
        let packet = self.create_telemetry_packet(sensor_data, nav_solution, flight_state);

        if let Ok(serialized) = postcard::to_vec::<_, 256>(&packet) {
            self.transmit_packet(&serialized);
        }

        self.packet_counter += 1;
    }

    pub fn send_abort_signal(&mut self) {
        let abort_packet = vec![0xFF; 16];
        self.transmit_packet(&abort_packet);
        defmt::error!("ABORT signal transmitted");
    }

    pub fn enable_recovery_beacon(&mut self) {
        self.recovery_beacon_enabled = true;
        defmt::info!("Recovery beacon enabled");
    }

    pub fn process_uplink(&mut self) -> Option<UplinkCommand> {
        if !self.config.uplink_enabled {
            return None;
        }

        if let Some(data) = self.receive_packet() {
            if let Ok(command) = postcard::from_bytes::<UplinkCommand>(&data) {
                defmt::info!("Received uplink command: {:?}", command);
                return Some(command);
            }
        }

        None
    }

    pub fn self_test(&mut self) -> bool {
        defmt::info!("Telemetry self-test");
        true
    }

    fn create_telemetry_packet(
        &self,
        sensor_data: &SensorData,
        nav_solution: &NavigationSolution,
        flight_state: FlightState,
    ) -> TelemetryPacket {
        TelemetryPacket {
            packet_id: self.packet_counter,
            timestamp_ms: sensor_data.timestamp_ms,
            flight_state: flight_state as u8,
            position: PositionData {
                latitude: (sensor_data.gps.latitude * 1e7) as i32,
                longitude: (sensor_data.gps.longitude * 1e7) as i32,
                altitude_m: nav_solution.position.z as i16,
                gps_valid: sensor_data.gps.valid,
            },
            velocity: VelocityData {
                velocity_north: (nav_solution.velocity.x * 10.0) as i16,
                velocity_east: (nav_solution.velocity.y * 10.0) as i16,
                velocity_down: (-nav_solution.velocity.z * 10.0) as i16,
            },
            attitude: AttitudeData {
                roll_deg: (nav_solution.attitude.x * 180.0 / core::f32::consts::PI) as i16,
                pitch_deg: (nav_solution.attitude.y * 180.0 / core::f32::consts::PI) as i16,
                yaw_deg: (nav_solution.attitude.z * 180.0 / core::f32::consts::PI) as i16,
            },
            sensors: CompressedSensorData {
                accel_x: (sensor_data.imu.accel.x * 100.0) as i16,
                accel_y: (sensor_data.imu.accel.y * 100.0) as i16,
                accel_z: (sensor_data.imu.accel.z * 100.0) as i16,
                gyro_x: (sensor_data.imu.gyro.x * 10.0) as i16,
                gyro_y: (sensor_data.imu.gyro.y * 10.0) as i16,
                gyro_z: (sensor_data.imu.gyro.z * 10.0) as i16,
                baro_altitude: sensor_data.barometer.altitude_m as i16,
                temperature: sensor_data.imu.temperature as i8,
            },
            system_health: SystemHealth {
                battery_voltage: 120,
                battery_current: 50,
                cpu_usage: 45,
                memory_free: 8192,
                sensor_status: 0xFF,
            },
            crc: 0,
        }
    }

    fn transmit_packet(&mut self, _data: &[u8]) {
    }

    fn receive_packet(&mut self) -> Option<Vec<u8, 256>> {
        None
    }
}