pub mod imu;
pub mod barometer;
pub mod gps;
pub mod temperature;
pub mod pressure;
pub mod analog;

use heapless::Vec;
use nalgebra::Vector3;

#[derive(Debug, Clone, Default)]
pub struct SensorData {
    pub timestamp_ms: u32,
    pub imu: ImuData,
    pub barometer: BarometerData,
    pub gps: GpsData,
    pub temperatures: Vec<TemperatureReading, 16>,
    pub pressures: Vec<PressureReading, 8>,
    pub analog_inputs: Vec<AnalogReading, 32>,
    pub sensor_status: SensorStatus,
}

#[derive(Debug, Clone, Default)]
pub struct ImuData {
    pub accel: Vector3<f32>,
    pub gyro: Vector3<f32>,
    pub mag: Vector3<f32>,
    pub temperature: f32,
}

#[derive(Debug, Clone, Default)]
pub struct BarometerData {
    pub pressure_pa: f32,
    pub altitude_m: f32,
    pub temperature_c: f32,
    pub vertical_velocity_mps: f32,
}

#[derive(Debug, Clone, Default)]
pub struct GpsData {
    pub valid: bool,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude_m: f32,
    pub velocity_ned: Vector3<f32>,
    pub hdop: f32,
    pub num_satellites: u8,
    pub fix_type: GpsFixType,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum GpsFixType {
    #[default]
    NoFix,
    Fix2D,
    Fix3D,
    FixRtk,
}

#[derive(Debug, Clone)]
pub struct TemperatureReading {
    pub sensor_id: u8,
    pub temperature_c: f32,
    pub location: SensorLocation,
}

#[derive(Debug, Clone)]
pub struct PressureReading {
    pub sensor_id: u8,
    pub pressure_pa: f32,
    pub location: SensorLocation,
}

#[derive(Debug, Clone)]
pub struct AnalogReading {
    pub channel: u8,
    pub raw_value: u16,
    pub scaled_value: f32,
    pub unit: AnalogUnit,
}

#[derive(Debug, Clone, Copy)]
pub enum SensorLocation {
    EngineChambered,
    TankOxidizer,
    TankFuel,
    Avionics,
    External,
    Nozzle,
    Injector,
}

#[derive(Debug, Clone, Copy)]
pub enum AnalogUnit {
    Voltage,
    Current,
    Pressure,
    Temperature,
    Force,
    Position,
}

#[derive(Debug, Clone, Default)]
pub struct SensorStatus {
    pub imu_ok: bool,
    pub baro_ok: bool,
    pub gps_ok: bool,
    pub temp_sensors_ok: u16,
    pub pressure_sensors_ok: u8,
    pub analog_ok: u32,
}

pub struct SensorManager {
    imu: imu::ImuSensor,
    barometer: barometer::Barometer,
    gps: gps::GpsReceiver,
    temperature_sensors: Vec<temperature::TemperatureSensor, 16>,
    pressure_sensors: Vec<pressure::PressureSensor, 8>,
    analog: analog::AnalogInputs,
    latest_data: SensorData,
    calibration: SensorCalibration,
}

#[derive(Debug, Clone, Default)]
pub struct SensorCalibration {
    pub accel_bias: Vector3<f32>,
    pub accel_scale: Vector3<f32>,
    pub gyro_bias: Vector3<f32>,
    pub mag_bias: Vector3<f32>,
    pub mag_scale: Vector3<f32>,
    pub baro_offset: f32,
}

impl SensorManager {
    pub fn new() -> Self {
        Self {
            imu: imu::ImuSensor::new(),
            barometer: barometer::Barometer::new(),
            gps: gps::GpsReceiver::new(),
            temperature_sensors: Vec::new(),
            pressure_sensors: Vec::new(),
            analog: analog::AnalogInputs::new(),
            latest_data: SensorData::default(),
            calibration: SensorCalibration::default(),
        }
    }

    pub fn initialize(&mut self) {
        self.imu.initialize();
        self.barometer.initialize();
        self.gps.initialize();
        self.analog.initialize();

        self.load_calibration();
    }

    pub fn update(&mut self) {
        let timestamp_ms = self.get_timestamp_ms();

        let mut data = SensorData {
            timestamp_ms,
            ..Default::default()
        };

        if let Ok(imu_raw) = self.imu.read() {
            data.imu = self.apply_imu_calibration(imu_raw);
            data.sensor_status.imu_ok = true;
        }

        if let Ok(baro) = self.barometer.read() {
            data.barometer = baro;
            data.barometer.pressure_pa += self.calibration.baro_offset;
            data.sensor_status.baro_ok = true;
        }

        if let Ok(gps) = self.gps.read() {
            data.gps = gps;
            data.sensor_status.gps_ok = true;
        }

        for sensor in &mut self.temperature_sensors {
            if let Ok(reading) = sensor.read() {
                let _ = data.temperatures.push(reading);
                data.sensor_status.temp_sensors_ok |= 1 << sensor.get_id();
            }
        }

        for sensor in &mut self.pressure_sensors {
            if let Ok(reading) = sensor.read() {
                let _ = data.pressures.push(reading);
                data.sensor_status.pressure_sensors_ok |= 1 << sensor.get_id();
            }
        }

        if let Ok(analog_readings) = self.analog.read_all() {
            data.analog_inputs = analog_readings;
            data.sensor_status.analog_ok = 0xFFFFFFFF;
        }

        self.latest_data = data;
    }

    pub fn get_latest_data(&self) -> SensorData {
        self.latest_data.clone()
    }

    pub fn self_test(&mut self) -> bool {
        let mut all_ok = true;

        all_ok &= self.imu.self_test();
        all_ok &= self.barometer.self_test();
        all_ok &= self.gps.self_test();
        all_ok &= self.analog.self_test();

        all_ok
    }

    fn apply_imu_calibration(&self, raw: ImuData) -> ImuData {
        ImuData {
            accel: Vector3::new(
                (raw.accel.x - self.calibration.accel_bias.x) * self.calibration.accel_scale.x,
                (raw.accel.y - self.calibration.accel_bias.y) * self.calibration.accel_scale.y,
                (raw.accel.z - self.calibration.accel_bias.z) * self.calibration.accel_scale.z,
            ),
            gyro: raw.gyro - self.calibration.gyro_bias,
            mag: Vector3::new(
                (raw.mag.x - self.calibration.mag_bias.x) * self.calibration.mag_scale.x,
                (raw.mag.y - self.calibration.mag_bias.y) * self.calibration.mag_scale.y,
                (raw.mag.z - self.calibration.mag_bias.z) * self.calibration.mag_scale.z,
            ),
            temperature: raw.temperature,
        }
    }

    fn load_calibration(&mut self) {
        self.calibration = SensorCalibration {
            accel_bias: Vector3::new(0.0, 0.0, 0.0),
            accel_scale: Vector3::new(1.0, 1.0, 1.0),
            gyro_bias: Vector3::new(0.0, 0.0, 0.0),
            mag_bias: Vector3::new(0.0, 0.0, 0.0),
            mag_scale: Vector3::new(1.0, 1.0, 1.0),
            baro_offset: 0.0,
        };
    }

    fn get_timestamp_ms(&self) -> u32 {
        0
    }
}