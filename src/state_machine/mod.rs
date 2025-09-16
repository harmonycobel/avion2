use crate::navigation::NavigationSolution;
use crate::sensors::SensorData;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightState {
    PreFlight,
    Armed,
    Ignition,
    Powered,
    Coast,
    Apogee,
    Descent,
    Landed,
    Abort,
    SafeMode,
}

pub struct StateMachine {
    current_state: FlightState,
    previous_state: FlightState,
    state_entry_time_ms: u32,
    launch_detected: bool,
    burnout_detected: bool,
    apogee_detected: bool,
    landing_detected: bool,
    launch_time_ms: Option<u32>,
    apogee_time_ms: Option<u32>,
    landing_time_ms: Option<u32>,
    state_transition_conditions: StateTransitionConditions,
}

#[derive(Clone)]
struct StateTransitionConditions {
    launch_accel_threshold_g: f32,
    launch_altitude_threshold_m: f32,
    burnout_accel_threshold_g: f32,
    apogee_velocity_threshold_mps: f32,
    landing_velocity_threshold_mps: f32,
    landing_altitude_threshold_m: f32,
    state_timeout_ms: u32,
}

impl Default for StateTransitionConditions {
    fn default() -> Self {
        Self {
            launch_accel_threshold_g: 3.0,
            launch_altitude_threshold_m: 10.0,
            burnout_accel_threshold_g: -0.5,
            apogee_velocity_threshold_mps: 5.0,
            landing_velocity_threshold_mps: 2.0,
            landing_altitude_threshold_m: 50.0,
            state_timeout_ms: 300000,
        }
    }
}

impl StateMachine {
    pub fn new() -> Self {
        Self {
            current_state: FlightState::PreFlight,
            previous_state: FlightState::PreFlight,
            state_entry_time_ms: 0,
            launch_detected: false,
            burnout_detected: false,
            apogee_detected: false,
            landing_detected: false,
            launch_time_ms: None,
            apogee_time_ms: None,
            landing_time_ms: None,
            state_transition_conditions: StateTransitionConditions::default(),
        }
    }

    pub fn update(
        &mut self,
        sensor_data: &SensorData,
        nav_solution: &NavigationSolution,
    ) -> FlightState {
        let new_state = self.evaluate_transitions(sensor_data, nav_solution);

        if new_state != self.current_state {
            self.transition_to(new_state, sensor_data.timestamp_ms);
        }

        self.current_state
    }

    pub fn get_current_state(&self) -> FlightState {
        self.current_state
    }

    pub fn force_state(&mut self, state: FlightState) {
        defmt::warn!("Forcing state transition to {:?}", state);
        self.transition_to(state, self.get_time_ms());
    }

    pub fn arm(&mut self) -> bool {
        if self.current_state == FlightState::PreFlight {
            self.transition_to(FlightState::Armed, self.get_time_ms());
            true
        } else {
            false
        }
    }

    pub fn disarm(&mut self) -> bool {
        if self.current_state == FlightState::Armed {
            self.transition_to(FlightState::PreFlight, self.get_time_ms());
            true
        } else {
            false
        }
    }

    fn evaluate_transitions(
        &mut self,
        sensor_data: &SensorData,
        nav_solution: &NavigationSolution,
    ) -> FlightState {
        match self.current_state {
            FlightState::PreFlight => {
                self.current_state
            }
            FlightState::Armed => {
                if self.check_ignition_command() {
                    FlightState::Ignition
                } else {
                    self.current_state
                }
            }
            FlightState::Ignition => {
                if self.detect_launch(sensor_data, nav_solution) {
                    FlightState::Powered
                } else if self.get_time_in_state_ms() > 5000 {
                    defmt::error!("Ignition timeout - no launch detected");
                    FlightState::SafeMode
                } else {
                    self.current_state
                }
            }
            FlightState::Powered => {
                if self.detect_burnout(sensor_data, nav_solution) {
                    FlightState::Coast
                } else if self.get_time_in_state_ms() > 180000 {
                    defmt::warn!("Maximum burn time exceeded");
                    FlightState::Coast
                } else {
                    self.current_state
                }
            }
            FlightState::Coast => {
                if self.detect_apogee(sensor_data, nav_solution) {
                    FlightState::Apogee
                } else if self.get_time_in_state_ms() > 60000 {
                    defmt::warn!("Coast timeout - forcing apogee");
                    FlightState::Apogee
                } else {
                    self.current_state
                }
            }
            FlightState::Apogee => {
                if self.get_time_in_state_ms() > 2000 {
                    FlightState::Descent
                } else {
                    self.current_state
                }
            }
            FlightState::Descent => {
                if self.detect_landing(sensor_data, nav_solution) {
                    FlightState::Landed
                } else {
                    self.current_state
                }
            }
            FlightState::Landed => {
                self.current_state
            }
            FlightState::Abort => {
                if self.detect_landing(sensor_data, nav_solution) {
                    FlightState::Landed
                } else {
                    self.current_state
                }
            }
            FlightState::SafeMode => {
                self.current_state
            }
        }
    }

    fn transition_to(&mut self, new_state: FlightState, timestamp_ms: u32) {
        defmt::info!(
            "State transition: {:?} -> {:?} at {} ms",
            self.current_state,
            new_state,
            timestamp_ms
        );

        self.previous_state = self.current_state;
        self.current_state = new_state;
        self.state_entry_time_ms = timestamp_ms;

        match new_state {
            FlightState::Powered => {
                self.launch_time_ms = Some(timestamp_ms);
                self.launch_detected = true;
            }
            FlightState::Apogee => {
                self.apogee_time_ms = Some(timestamp_ms);
                self.apogee_detected = true;
            }
            FlightState::Landed => {
                self.landing_time_ms = Some(timestamp_ms);
                self.landing_detected = true;
            }
            _ => {}
        }
    }

    fn detect_launch(
        &self,
        sensor_data: &SensorData,
        nav_solution: &NavigationSolution,
    ) -> bool {
        let accel_magnitude = sensor_data.imu.accel.magnitude() / 9.81;
        let altitude = nav_solution.position.z;

        accel_magnitude > self.state_transition_conditions.launch_accel_threshold_g
            && altitude > self.state_transition_conditions.launch_altitude_threshold_m
    }

    fn detect_burnout(
        &mut self,
        sensor_data: &SensorData,
        _nav_solution: &NavigationSolution,
    ) -> bool {
        let accel_z = sensor_data.imu.accel.z / 9.81;

        if accel_z < self.state_transition_conditions.burnout_accel_threshold_g {
            if !self.burnout_detected {
                defmt::info!("Burnout detected - acceleration dropped below threshold");
                self.burnout_detected = true;
            }
            true
        } else {
            false
        }
    }

    fn detect_apogee(
        &mut self,
        _sensor_data: &SensorData,
        nav_solution: &NavigationSolution,
    ) -> bool {
        let vertical_velocity = nav_solution.velocity.z.abs();

        if vertical_velocity < self.state_transition_conditions.apogee_velocity_threshold_mps
            && nav_solution.position.z > 100.0
        {
            if !self.apogee_detected {
                defmt::info!("Apogee detected at {} m", nav_solution.position.z);
                self.apogee_detected = true;
            }
            true
        } else {
            false
        }
    }

    fn detect_landing(
        &mut self,
        _sensor_data: &SensorData,
        nav_solution: &NavigationSolution,
    ) -> bool {
        let velocity = nav_solution.velocity.magnitude();
        let altitude = nav_solution.position.z;

        if velocity < self.state_transition_conditions.landing_velocity_threshold_mps
            && altitude < self.state_transition_conditions.landing_altitude_threshold_m
        {
            if !self.landing_detected {
                defmt::info!("Landing detected");
                self.landing_detected = true;
            }
            true
        } else {
            false
        }
    }

    fn check_ignition_command(&self) -> bool {
        false
    }

    fn get_time_in_state_ms(&self) -> u32 {
        self.get_time_ms() - self.state_entry_time_ms
    }

    fn get_time_ms(&self) -> u32 {
        0
    }

    pub fn get_mission_elapsed_time_ms(&self) -> Option<u32> {
        self.launch_time_ms.map(|launch| self.get_time_ms() - launch)
    }
}