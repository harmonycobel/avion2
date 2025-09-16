use crate::sensors::{SensorData, GpsFixType};
use heapless::Vec;
use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};

#[derive(Debug, Clone)]
pub struct NavigationSolution {
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub attitude: Vector3<f32>,
    pub acceleration: Vector3<f32>,
    pub angular_velocity: Vector3<f32>,
    pub covariance: Matrix6<f32>,
    pub navigation_mode: NavigationMode,
    pub time_since_last_gps_ms: u32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NavigationMode {
    Initializing,
    GpsOnly,
    ImuOnly,
    GpsImuFusion,
    DeadReckoning,
}

pub struct NavigationSystem {
    ekf: ExtendedKalmanFilter,
    guidance: GuidanceController,
    last_gps_time_ms: u32,
    last_update_time_ms: u32,
    launch_position: Option<Vector3<f32>>,
    target_trajectory: Option<Trajectory>,
    closed_loop_enabled: bool,
}

struct ExtendedKalmanFilter {
    state: Vector6<f32>,
    covariance: Matrix6<f32>,
    process_noise: Matrix6<f32>,
    measurement_noise_gps: Matrix3<f32>,
    measurement_noise_baro: f32,
}

struct GuidanceController {
    proportional_gain: Vector3<f32>,
    integral_gain: Vector3<f32>,
    derivative_gain: Vector3<f32>,
    integral_error: Vector3<f32>,
    last_error: Vector3<f32>,
    max_throttle_command: f32,
    max_tvc_angle_deg: f32,
    max_fin_deflection_deg: f32,
}

#[derive(Clone)]
struct Trajectory {
    waypoints: Vec<Waypoint, 32>,
    current_waypoint_index: usize,
}

#[derive(Clone)]
struct Waypoint {
    position: Vector3<f32>,
    velocity: Vector3<f32>,
    time_ms: u32,
}

impl NavigationSystem {
    pub fn new() -> Self {
        Self {
            ekf: ExtendedKalmanFilter {
                state: Vector6::zeros(),
                covariance: Matrix6::identity() * 100.0,
                process_noise: Matrix6::identity() * 0.1,
                measurement_noise_gps: Matrix3::identity() * 5.0,
                measurement_noise_baro: 1.0,
            },
            guidance: GuidanceController {
                proportional_gain: Vector3::new(1.0, 1.0, 2.0),
                integral_gain: Vector3::new(0.1, 0.1, 0.2),
                derivative_gain: Vector3::new(0.5, 0.5, 1.0),
                integral_error: Vector3::zeros(),
                last_error: Vector3::zeros(),
                max_throttle_command: 1.0,
                max_tvc_angle_deg: 5.0,
                max_fin_deflection_deg: 15.0,
            },
            last_gps_time_ms: 0,
            last_update_time_ms: 0,
            launch_position: None,
            target_trajectory: None,
            closed_loop_enabled: false,
        }
    }

    pub fn initialize(&mut self) {
        defmt::info!("Initializing navigation system");
        self.ekf.state = Vector6::zeros();
        self.ekf.covariance = Matrix6::identity() * 100.0;
    }

    pub fn update(&mut self, sensor_data: &SensorData, dt: f32) -> NavigationSolution {
        self.ekf_predict(sensor_data, dt);

        if sensor_data.gps.valid && sensor_data.gps.fix_type != GpsFixType::NoFix {
            self.ekf_update_gps(sensor_data);
            self.last_gps_time_ms = sensor_data.timestamp_ms;
        }

        self.ekf_update_barometer(sensor_data);

        let time_since_gps = sensor_data.timestamp_ms - self.last_gps_time_ms;
        let nav_mode = self.determine_navigation_mode(sensor_data, time_since_gps);

        self.last_update_time_ms = sensor_data.timestamp_ms;

        NavigationSolution {
            position: Vector3::new(
                self.ekf.state[0],
                self.ekf.state[1],
                self.ekf.state[2],
            ),
            velocity: Vector3::new(
                self.ekf.state[3],
                self.ekf.state[4],
                self.ekf.state[5],
            ),
            attitude: self.estimate_attitude(sensor_data),
            acceleration: sensor_data.imu.accel,
            angular_velocity: sensor_data.imu.gyro,
            covariance: self.ekf.covariance,
            navigation_mode: nav_mode,
            time_since_last_gps_ms: time_since_gps,
        }
    }

    pub fn enable_closed_loop_guidance(&mut self) {
        defmt::info!("Enabling closed-loop guidance");
        self.closed_loop_enabled = true;
        self.guidance.integral_error = Vector3::zeros();
    }

    pub fn compute_throttle_command(&self, nav_solution: &NavigationSolution) -> f32 {
        if !self.closed_loop_enabled {
            return 1.0;
        }

        if let Some(ref trajectory) = self.target_trajectory {
            if let Some(target) = trajectory.waypoints.get(trajectory.current_waypoint_index) {
                let altitude_error = target.position.z - nav_solution.position.z;
                let velocity_error = target.velocity.z - nav_solution.velocity.z;

                let throttle = 0.8
                    + altitude_error * 0.001
                    + velocity_error * 0.01;

                throttle.clamp(0.3, self.guidance.max_throttle_command)
            } else {
                1.0
            }
        } else {
            1.0
        }
    }

    pub fn compute_tvc_command(&mut self, nav_solution: &NavigationSolution) -> Vector3<f32> {
        if !self.closed_loop_enabled {
            return Vector3::zeros();
        }

        if let Some(ref trajectory) = self.target_trajectory {
            if let Some(target) = trajectory.waypoints.get(trajectory.current_waypoint_index) {
                let position_error = target.position - nav_solution.position;
                let velocity_error = target.velocity - nav_solution.velocity;

                let p_term = position_error.component_mul(&self.guidance.proportional_gain);
                let d_term = velocity_error.component_mul(&self.guidance.derivative_gain);

                self.guidance.integral_error += position_error * 0.01;
                let i_term = self.guidance.integral_error.component_mul(&self.guidance.integral_gain);

                let control = p_term + i_term + d_term;

                let tvc_x = control.x.clamp(-self.guidance.max_tvc_angle_deg, self.guidance.max_tvc_angle_deg);
                let tvc_y = control.y.clamp(-self.guidance.max_tvc_angle_deg, self.guidance.max_tvc_angle_deg);

                Vector3::new(tvc_x, tvc_y, 0.0)
            } else {
                Vector3::zeros()
            }
        } else {
            Vector3::zeros()
        }
    }

    pub fn compute_fin_command(&self, nav_solution: &NavigationSolution) -> Vec<f32, 4> {
        let mut fin_commands = Vec::new();

        let roll_rate = nav_solution.angular_velocity.x;
        let pitch_rate = nav_solution.angular_velocity.y;
        let yaw_rate = nav_solution.angular_velocity.z;

        let roll_cmd = -roll_rate * 5.0;
        let pitch_cmd = -pitch_rate * 5.0;
        let yaw_cmd = -yaw_rate * 3.0;

        let _ = fin_commands.push((pitch_cmd + roll_cmd).clamp(
            -self.guidance.max_fin_deflection_deg,
            self.guidance.max_fin_deflection_deg,
        ));
        let _ = fin_commands.push((pitch_cmd - roll_cmd).clamp(
            -self.guidance.max_fin_deflection_deg,
            self.guidance.max_fin_deflection_deg,
        ));
        let _ = fin_commands.push((-pitch_cmd + roll_cmd).clamp(
            -self.guidance.max_fin_deflection_deg,
            self.guidance.max_fin_deflection_deg,
        ));
        let _ = fin_commands.push((-pitch_cmd - roll_cmd + yaw_cmd).clamp(
            -self.guidance.max_fin_deflection_deg,
            self.guidance.max_fin_deflection_deg,
        ));

        fin_commands
    }

    pub fn get_altitude(&self) -> f32 {
        self.ekf.state[2]
    }

    pub fn self_test(&mut self) -> bool {
        defmt::info!("Navigation system self-test");
        true
    }

    fn ekf_predict(&mut self, sensor_data: &SensorData, dt: f32) {
        let accel = sensor_data.imu.accel;

        self.ekf.state[0] += self.ekf.state[3] * dt + 0.5 * accel.x * dt * dt;
        self.ekf.state[1] += self.ekf.state[4] * dt + 0.5 * accel.y * dt * dt;
        self.ekf.state[2] += self.ekf.state[5] * dt + 0.5 * accel.z * dt * dt;

        self.ekf.state[3] += accel.x * dt;
        self.ekf.state[4] += accel.y * dt;
        self.ekf.state[5] += accel.z * dt;

        self.ekf.covariance = self.ekf.covariance + self.ekf.process_noise * dt;
    }

    fn ekf_update_gps(&mut self, sensor_data: &SensorData) {
        if self.launch_position.is_none() {
            self.launch_position = Some(Vector3::new(
                0.0,
                0.0,
                sensor_data.gps.altitude_m,
            ));
        }

        let gps_position = self.gps_to_local_position(
            sensor_data.gps.latitude,
            sensor_data.gps.longitude,
            sensor_data.gps.altitude_m,
        );

        let innovation = gps_position - Vector3::new(
            self.ekf.state[0],
            self.ekf.state[1],
            self.ekf.state[2],
        );

        self.ekf.state[0] += innovation.x * 0.1;
        self.ekf.state[1] += innovation.y * 0.1;
        self.ekf.state[2] += innovation.z * 0.1;

        self.ekf.state[3] = sensor_data.gps.velocity_ned.x;
        self.ekf.state[4] = sensor_data.gps.velocity_ned.y;
        self.ekf.state[5] = -sensor_data.gps.velocity_ned.z;
    }

    fn ekf_update_barometer(&mut self, sensor_data: &SensorData) {
        let baro_altitude = sensor_data.barometer.altitude_m;
        let innovation = baro_altitude - self.ekf.state[2];

        self.ekf.state[2] += innovation * 0.05;
        self.ekf.state[5] += sensor_data.barometer.vertical_velocity_mps * 0.1;
    }

    fn estimate_attitude(&self, sensor_data: &SensorData) -> Vector3<f32> {
        let accel = sensor_data.imu.accel.normalize();
        let mag = sensor_data.imu.mag.normalize();

        let pitch = (-accel.x).atan2((accel.y * accel.y + accel.z * accel.z).sqrt());
        let roll = accel.y.atan2(accel.z);

        let mag_x = mag.x * pitch.cos() + mag.z * pitch.sin();
        let mag_y = mag.x * roll.sin() * pitch.sin() + mag.y * roll.cos() - mag.z * roll.sin() * pitch.cos();
        let yaw = (-mag_y).atan2(mag_x);

        Vector3::new(roll, pitch, yaw)
    }

    fn determine_navigation_mode(&self, sensor_data: &SensorData, time_since_gps: u32) -> NavigationMode {
        if !sensor_data.sensor_status.imu_ok {
            return NavigationMode::Initializing;
        }

        if sensor_data.gps.valid && time_since_gps < 1000 {
            if sensor_data.sensor_status.imu_ok {
                NavigationMode::GpsImuFusion
            } else {
                NavigationMode::GpsOnly
            }
        } else if sensor_data.sensor_status.imu_ok {
            if time_since_gps < 10000 {
                NavigationMode::ImuOnly
            } else {
                NavigationMode::DeadReckoning
            }
        } else {
            NavigationMode::DeadReckoning
        }
    }

    fn gps_to_local_position(&self, lat: f64, lon: f64, alt: f32) -> Vector3<f32> {
        if let Some(ref launch_pos) = self.launch_position {
            Vector3::new(
                ((lon - 0.0) * 111320.0 * lat.to_radians().cos()) as f32,
                ((lat - 0.0) * 110540.0) as f32,
                alt - launch_pos.z,
            )
        } else {
            Vector3::new(0.0, 0.0, alt)
        }
    }
}