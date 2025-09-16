use crate::config::SystemConfig;
use crate::data_logger::DataLogger;
use crate::engine::EngineController;
use crate::navigation::NavigationSystem;
use crate::safety::SafetyMonitor;
use crate::sensors::SensorManager;
use crate::state_machine::{FlightState, StateMachine};
use crate::telemetry::TelemetrySystem;
use heapless::Vec;

pub struct FlightComputer {
    config: SystemConfig,
    state_machine: StateMachine,
    sensor_manager: SensorManager,
    engine_controller: EngineController,
    navigation: NavigationSystem,
    telemetry: TelemetrySystem,
    data_logger: DataLogger,
    safety_monitor: SafetyMonitor,
    cycle_counter: u32,
    last_update_ms: u32,
}

impl FlightComputer {
    pub fn new(config: SystemConfig) -> Self {
        Self {
            config: config.clone(),
            state_machine: StateMachine::new(),
            sensor_manager: SensorManager::new(),
            engine_controller: EngineController::new(config.engine.clone()),
            navigation: NavigationSystem::new(),
            telemetry: TelemetrySystem::new(config.telemetry.clone()),
            data_logger: DataLogger::new(config.logging.clone()),
            safety_monitor: SafetyMonitor::new(config.safety.clone()),
            cycle_counter: 0,
            last_update_ms: 0,
        }
    }

    pub fn initialize(&mut self) {
        defmt::info!("Initializing flight computer subsystems...");

        self.sensor_manager.initialize();
        self.engine_controller.initialize();
        self.navigation.initialize();
        self.telemetry.initialize();
        self.data_logger.initialize();
        self.safety_monitor.initialize();

        self.perform_self_test();

        defmt::info!("Flight computer initialization complete");
    }

    pub fn update(&mut self) {
        let current_time_ms = self.get_system_time_ms();
        let dt = (current_time_ms - self.last_update_ms) as f32 / 1000.0;
        self.last_update_ms = current_time_ms;

        self.sensor_manager.update();
        let sensor_data = self.sensor_manager.get_latest_data();

        self.safety_monitor.check(&sensor_data);
        if self.safety_monitor.is_abort_required() {
            self.handle_abort();
            return;
        }

        let nav_solution = self.navigation.update(&sensor_data, dt);

        let current_state = self.state_machine.get_current_state();
        let new_state = self.state_machine.update(&sensor_data, &nav_solution);

        if new_state != current_state {
            self.handle_state_transition(current_state, new_state);
        }

        self.execute_state_logic(new_state, &sensor_data, &nav_solution);

        self.engine_controller.update(new_state, &sensor_data);

        if self.cycle_counter % self.config.telemetry.downlink_rate_hz == 0 {
            self.telemetry.send_telemetry(&sensor_data, &nav_solution, new_state);
        }

        self.data_logger.log_frame(&sensor_data, &nav_solution, new_state);

        self.cycle_counter += 1;
    }

    fn handle_state_transition(&mut self, from: FlightState, to: FlightState) {
        defmt::info!("State transition: {:?} -> {:?}", from, to);

        match to {
            FlightState::Armed => {
                self.engine_controller.arm();
                self.data_logger.start_recording();
            }
            FlightState::Ignition => {
                self.engine_controller.ignite();
            }
            FlightState::Powered => {
                self.navigation.enable_closed_loop_guidance();
            }
            FlightState::Coast => {
                self.engine_controller.shutdown();
            }
            FlightState::Apogee => {
                self.deploy_recovery_system(1);
            }
            FlightState::Descent => {
                if self.navigation.get_altitude() < self.config.recovery.main_deploy_altitude {
                    self.deploy_recovery_system(2);
                }
            }
            FlightState::Landed => {
                self.data_logger.stop_recording();
                self.telemetry.enable_recovery_beacon();
            }
            _ => {}
        }
    }

    fn execute_state_logic(
        &mut self,
        state: FlightState,
        sensor_data: &crate::sensors::SensorData,
        nav_solution: &crate::navigation::NavigationSolution,
    ) {
        match state {
            FlightState::Powered => {
                let throttle_cmd = self.navigation.compute_throttle_command(nav_solution);
                self.engine_controller.set_throttle(throttle_cmd);

                let tvc_cmd = self.navigation.compute_tvc_command(nav_solution);
                self.engine_controller.set_tvc_position(tvc_cmd.x, tvc_cmd.y);
            }
            FlightState::Coast => {
                let fin_cmd = self.navigation.compute_fin_command(nav_solution);
                self.engine_controller.set_fin_positions(fin_cmd);
            }
            _ => {}
        }
    }

    fn handle_abort(&mut self) {
        defmt::error!("ABORT INITIATED");
        self.state_machine.force_state(FlightState::Abort);
        self.engine_controller.emergency_shutdown();
        self.deploy_recovery_system(0);
        self.telemetry.send_abort_signal();
    }

    fn deploy_recovery_system(&mut self, stage: u8) {
        defmt::info!("Deploying recovery system stage {}", stage);
    }

    fn perform_self_test(&mut self) -> bool {
        defmt::info!("Performing system self-test...");

        let mut tests_passed = true;

        tests_passed &= self.sensor_manager.self_test();
        tests_passed &= self.engine_controller.self_test();
        tests_passed &= self.navigation.self_test();
        tests_passed &= self.telemetry.self_test();
        tests_passed &= self.data_logger.self_test();

        if tests_passed {
            defmt::info!("All self-tests passed");
        } else {
            defmt::error!("Self-test failures detected");
        }

        tests_passed
    }

    fn get_system_time_ms(&self) -> u32 {
        0
    }
}