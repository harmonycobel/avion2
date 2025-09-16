use crate::sensors::SensorData;
use crate::state_machine::FlightState;
use heapless::Vec;
use nalgebra::Vector2;

#[derive(Clone)]
pub struct EngineConfig {
    pub thrust_curve: Vec<(f32, f32), 32>,
    pub max_thrust_n: f32,
    pub burn_time_s: f32,
    pub throttle_response_hz: f32,
    pub tvc_enabled: bool,
    pub tvc_max_angle_deg: f32,
    pub fin_control_enabled: bool,
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self {
            thrust_curve: Vec::new(),
            max_thrust_n: 10000.0,
            burn_time_s: 120.0,
            throttle_response_hz: 50.0,
            tvc_enabled: true,
            tvc_max_angle_deg: 5.0,
            fin_control_enabled: true,
        }
    }
}

pub struct EngineController {
    config: EngineConfig,
    armed: bool,
    ignited: bool,
    throttle_command: f32,
    tvc_command: Vector2<f32>,
    fin_positions: Vec<f32, 4>,
    ignition_time_ms: Option<u32>,
    valve_controller: ValveController,
    injector_controller: InjectorController,
    igniter: Igniter,
    tvc_actuators: TvcActuators,
    fin_servos: FinServos,
}

struct ValveController {
    fuel_valve_open: bool,
    oxidizer_valve_open: bool,
    pressurant_valve_open: bool,
    vent_valve_open: bool,
}

struct InjectorController {
    injector_pressure_pa: f32,
    target_mixture_ratio: f32,
}

struct Igniter {
    spark_enabled: bool,
    torch_enabled: bool,
    ignition_attempts: u8,
}

struct TvcActuators {
    enabled: bool,
    x_position: f32,
    y_position: f32,
    max_rate_deg_per_s: f32,
}

struct FinServos {
    enabled: bool,
    positions: Vec<f32, 4>,
    max_deflection_deg: f32,
}

impl EngineController {
    pub fn new(config: EngineConfig) -> Self {
        Self {
            config,
            armed: false,
            ignited: false,
            throttle_command: 0.0,
            tvc_command: Vector2::new(0.0, 0.0),
            fin_positions: Vec::new(),
            ignition_time_ms: None,
            valve_controller: ValveController {
                fuel_valve_open: false,
                oxidizer_valve_open: false,
                pressurant_valve_open: false,
                vent_valve_open: false,
            },
            injector_controller: InjectorController {
                injector_pressure_pa: 0.0,
                target_mixture_ratio: 2.8,
            },
            igniter: Igniter {
                spark_enabled: false,
                torch_enabled: false,
                ignition_attempts: 0,
            },
            tvc_actuators: TvcActuators {
                enabled: false,
                x_position: 0.0,
                y_position: 0.0,
                max_rate_deg_per_s: 30.0,
            },
            fin_servos: FinServos {
                enabled: false,
                positions: Vec::new(),
                max_deflection_deg: 15.0,
            },
        }
    }

    pub fn initialize(&mut self) {
        defmt::info!("Initializing engine controller");

        self.valve_controller.vent_valve_open = true;

        if self.config.tvc_enabled {
            self.tvc_actuators.enabled = true;
            defmt::info!("TVC system enabled");
        }

        if self.config.fin_control_enabled {
            self.fin_servos.enabled = true;
            let _ = self.fin_servos.positions.push(0.0).ok();
            let _ = self.fin_servos.positions.push(0.0).ok();
            let _ = self.fin_servos.positions.push(0.0).ok();
            let _ = self.fin_servos.positions.push(0.0).ok();
            defmt::info!("Fin control enabled");
        }
    }

    pub fn update(&mut self, state: FlightState, sensor_data: &SensorData) {
        match state {
            FlightState::Armed => {
                self.maintain_ready_state();
            }
            FlightState::Ignition => {
                self.execute_ignition_sequence();
            }
            FlightState::Powered => {
                self.control_thrust();
                self.update_tvc();
                self.monitor_combustion(sensor_data);
            }
            FlightState::Coast | FlightState::Descent => {
                self.update_fin_control();
            }
            _ => {}
        }
    }

    pub fn arm(&mut self) {
        if !self.armed {
            defmt::info!("Arming engine controller");
            self.armed = true;
            self.valve_controller.pressurant_valve_open = true;
            self.valve_controller.vent_valve_open = false;
        }
    }

    pub fn ignite(&mut self) {
        if self.armed && !self.ignited {
            defmt::info!("Initiating ignition sequence");
            self.ignited = true;
            self.ignition_time_ms = Some(self.get_time_ms());
            self.igniter.spark_enabled = true;
            self.igniter.torch_enabled = true;
        }
    }

    pub fn shutdown(&mut self) {
        defmt::info!("Engine shutdown commanded");
        self.throttle_command = 0.0;
        self.valve_controller.fuel_valve_open = false;
        self.valve_controller.oxidizer_valve_open = false;
        self.igniter.spark_enabled = false;
        self.igniter.torch_enabled = false;
    }

    pub fn emergency_shutdown(&mut self) {
        defmt::error!("EMERGENCY ENGINE SHUTDOWN");
        self.shutdown();
        self.valve_controller.pressurant_valve_open = false;
        self.valve_controller.vent_valve_open = true;
        self.armed = false;
        self.ignited = false;
    }

    pub fn set_throttle(&mut self, throttle: f32) {
        self.throttle_command = throttle.clamp(0.0, 1.0);
    }

    pub fn set_tvc_position(&mut self, x: f32, y: f32) {
        if self.tvc_actuators.enabled {
            let max_angle = self.config.tvc_max_angle_deg;
            self.tvc_command.x = x.clamp(-max_angle, max_angle);
            self.tvc_command.y = y.clamp(-max_angle, max_angle);
        }
    }

    pub fn set_fin_positions(&mut self, positions: Vec<f32, 4>) {
        if self.fin_servos.enabled {
            self.fin_positions = positions;
        }
    }

    pub fn self_test(&mut self) -> bool {
        defmt::info!("Engine controller self-test");

        let mut tests_ok = true;

        tests_ok &= self.test_valves();
        tests_ok &= self.test_tvc();
        tests_ok &= self.test_fins();
        tests_ok &= self.test_igniter();

        tests_ok
    }

    fn maintain_ready_state(&mut self) {
    }

    fn execute_ignition_sequence(&mut self) {
        self.valve_controller.fuel_valve_open = true;
        self.valve_controller.oxidizer_valve_open = true;

        self.injector_controller.injector_pressure_pa = 2000000.0;
    }

    fn control_thrust(&mut self) {
        let target_pressure = self.throttle_command * 5000000.0;
        self.injector_controller.injector_pressure_pa = target_pressure;
    }

    fn update_tvc(&mut self) {
        if self.tvc_actuators.enabled {
            let rate_limit = self.tvc_actuators.max_rate_deg_per_s / 100.0;

            let x_error = self.tvc_command.x - self.tvc_actuators.x_position;
            let y_error = self.tvc_command.y - self.tvc_actuators.y_position;

            self.tvc_actuators.x_position += x_error.clamp(-rate_limit, rate_limit);
            self.tvc_actuators.y_position += y_error.clamp(-rate_limit, rate_limit);
        }
    }

    fn update_fin_control(&mut self) {
        if self.fin_servos.enabled {
            for (i, &target) in self.fin_positions.iter().enumerate() {
                if let Some(current) = self.fin_servos.positions.get_mut(i) {
                    *current = target.clamp(
                        -self.fin_servos.max_deflection_deg,
                        self.fin_servos.max_deflection_deg,
                    );
                }
            }
        }
    }

    fn monitor_combustion(&mut self, sensor_data: &SensorData) {
        for pressure in &sensor_data.pressures {
            if matches!(pressure.location, crate::sensors::SensorLocation::EngineChambered) {
                if pressure.pressure_pa < 500000.0 {
                    defmt::warn!("Low combustion chamber pressure detected");
                }
            }
        }
    }

    fn test_valves(&mut self) -> bool {
        true
    }

    fn test_tvc(&mut self) -> bool {
        if self.config.tvc_enabled {
            defmt::debug!("Testing TVC actuators");
        }
        true
    }

    fn test_fins(&mut self) -> bool {
        if self.config.fin_control_enabled {
            defmt::debug!("Testing fin servos");
        }
        true
    }

    fn test_igniter(&mut self) -> bool {
        defmt::debug!("Testing ignition system");
        true
    }

    fn get_time_ms(&self) -> u32 {
        0
    }
}