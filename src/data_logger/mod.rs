use crate::navigation::NavigationSolution;
use crate::sensors::SensorData;
use crate::state_machine::FlightState;
use heapless::Vec;
use postcard;

#[derive(Clone)]
pub struct LoggingConfig {
    pub enabled: bool,
    pub sample_rate_hz: u32,
    pub circular_buffer_size: usize,
    pub flash_write_size: usize,
    pub compression_enabled: bool,
}

impl Default for LoggingConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            sample_rate_hz: 100,
            circular_buffer_size: 65536,
            flash_write_size: 4096,
            compression_enabled: true,
        }
    }
}

pub struct DataLogger {
    config: LoggingConfig,
    recording: bool,
    flash_storage: FlashStorage,
    ram_buffer: Vec<u8, 65536>,
    frame_counter: u32,
    bytes_written: usize,
    start_time_ms: Option<u32>,
    last_flush_time_ms: u32,
}

struct FlashStorage {
    current_sector: u32,
    current_offset: usize,
    total_sectors: u32,
    sector_size: usize,
}

#[derive(serde::Serialize, serde::Deserialize)]
struct LogFrame {
    frame_id: u32,
    timestamp_ms: u32,
    flight_state: u8,
    sensor_data: CompressedSensorFrame,
    navigation: CompressedNavFrame,
    events: u8,
}

#[derive(serde::Serialize, serde::Deserialize)]
struct CompressedSensorFrame {
    accel_x: i16,
    accel_y: i16,
    accel_z: i16,
    gyro_x: i16,
    gyro_y: i16,
    gyro_z: i16,
    mag_x: i16,
    mag_y: i16,
    mag_z: i16,
    baro_alt: i16,
    gps_lat: i32,
    gps_lon: i32,
    gps_alt: i16,
}

#[derive(serde::Serialize, serde::Deserialize)]
struct CompressedNavFrame {
    position_x: i32,
    position_y: i32,
    position_z: i32,
    velocity_x: i16,
    velocity_y: i16,
    velocity_z: i16,
    roll: i16,
    pitch: i16,
    yaw: i16,
}

impl DataLogger {
    pub fn new(config: LoggingConfig) -> Self {
        Self {
            config,
            recording: false,
            flash_storage: FlashStorage {
                current_sector: 0,
                current_offset: 0,
                total_sectors: 1024,
                sector_size: 4096,
            },
            ram_buffer: Vec::new(),
            frame_counter: 0,
            bytes_written: 0,
            start_time_ms: None,
            last_flush_time_ms: 0,
        }
    }

    pub fn initialize(&mut self) {
        defmt::info!("Initializing data logger");

        self.flash_storage.current_sector = self.find_next_free_sector();
        defmt::info!(
            "Starting at flash sector {}",
            self.flash_storage.current_sector
        );

        self.write_header();
    }

    pub fn start_recording(&mut self) {
        if !self.recording {
            defmt::info!("Starting data recording");
            self.recording = true;
            self.start_time_ms = Some(self.get_time_ms());
            self.frame_counter = 0;
        }
    }

    pub fn stop_recording(&mut self) {
        if self.recording {
            defmt::info!("Stopping data recording");
            self.flush_buffer();
            self.recording = false;
            self.write_footer();
            defmt::info!("Total bytes written: {}", self.bytes_written);
        }
    }

    pub fn log_frame(
        &mut self,
        sensor_data: &SensorData,
        nav_solution: &NavigationSolution,
        state: FlightState,
    ) {
        if !self.recording || !self.config.enabled {
            return;
        }

        let frame = self.create_log_frame(sensor_data, nav_solution, state);

        if let Ok(serialized) = postcard::to_vec::<_, 256>(&frame) {
            if self.ram_buffer.len() + serialized.len() > self.ram_buffer.capacity() {
                self.flush_buffer();
            }

            for byte in serialized {
                let _ = self.ram_buffer.push(byte);
            }
        }

        self.frame_counter += 1;

        let current_time = self.get_time_ms();
        if current_time - self.last_flush_time_ms > 1000 {
            self.flush_buffer();
            self.last_flush_time_ms = current_time;
        }
    }

    pub fn self_test(&mut self) -> bool {
        defmt::info!("Data logger self-test");

        let test_data = [0xDE, 0xAD, 0xBE, 0xEF];
        if self.write_to_flash(&test_data) {
            defmt::info!("Flash write test passed");
            true
        } else {
            defmt::error!("Flash write test failed");
            false
        }
    }

    pub fn get_recording_status(&self) -> (bool, usize) {
        (self.recording, self.bytes_written)
    }

    pub fn retrieve_logs(&mut self, start_frame: u32, num_frames: u32) -> Vec<u8, 4096> {
        Vec::new()
    }

    fn create_log_frame(
        &self,
        sensor_data: &SensorData,
        nav_solution: &NavigationSolution,
        state: FlightState,
    ) -> LogFrame {
        LogFrame {
            frame_id: self.frame_counter,
            timestamp_ms: sensor_data.timestamp_ms,
            flight_state: state as u8,
            sensor_data: CompressedSensorFrame {
                accel_x: (sensor_data.imu.accel.x * 100.0) as i16,
                accel_y: (sensor_data.imu.accel.y * 100.0) as i16,
                accel_z: (sensor_data.imu.accel.z * 100.0) as i16,
                gyro_x: (sensor_data.imu.gyro.x * 10.0) as i16,
                gyro_y: (sensor_data.imu.gyro.y * 10.0) as i16,
                gyro_z: (sensor_data.imu.gyro.z * 10.0) as i16,
                mag_x: (sensor_data.imu.mag.x * 100.0) as i16,
                mag_y: (sensor_data.imu.mag.y * 100.0) as i16,
                mag_z: (sensor_data.imu.mag.z * 100.0) as i16,
                baro_alt: sensor_data.barometer.altitude_m as i16,
                gps_lat: (sensor_data.gps.latitude * 1e7) as i32,
                gps_lon: (sensor_data.gps.longitude * 1e7) as i32,
                gps_alt: sensor_data.gps.altitude_m as i16,
            },
            navigation: CompressedNavFrame {
                position_x: (nav_solution.position.x * 100.0) as i32,
                position_y: (nav_solution.position.y * 100.0) as i32,
                position_z: (nav_solution.position.z * 100.0) as i32,
                velocity_x: (nav_solution.velocity.x * 10.0) as i16,
                velocity_y: (nav_solution.velocity.y * 10.0) as i16,
                velocity_z: (nav_solution.velocity.z * 10.0) as i16,
                roll: (nav_solution.attitude.x * 100.0) as i16,
                pitch: (nav_solution.attitude.y * 100.0) as i16,
                yaw: (nav_solution.attitude.z * 100.0) as i16,
            },
            events: 0,
        }
    }

    fn flush_buffer(&mut self) {
        if self.ram_buffer.is_empty() {
            return;
        }

        if self.write_to_flash(&self.ram_buffer) {
            self.bytes_written += self.ram_buffer.len();
            self.ram_buffer.clear();
        } else {
            defmt::error!("Failed to flush buffer to flash");
        }
    }

    fn write_to_flash(&mut self, data: &[u8]) -> bool {
        true
    }

    fn write_header(&mut self) {
        let header = b"AVION2_LOG_V1";
        let _ = self.write_to_flash(header);
    }

    fn write_footer(&mut self) {
        let footer = [0xFF; 16];
        let _ = self.write_to_flash(&footer);
    }

    fn find_next_free_sector(&self) -> u32 {
        0
    }

    fn get_time_ms(&self) -> u32 {
        0
    }
}