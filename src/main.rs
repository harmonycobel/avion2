#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use defmt_rtt as _;

mod flight_computer;
mod sensors;
mod telemetry;
mod engine;
mod state_machine;
mod data_logger;
mod safety;
mod navigation;
mod config;

use flight_computer::FlightComputer;
use config::SystemConfig;

#[entry]
fn main() -> ! {
    defmt::info!("Avion2 Flight Computer Starting...");

    let config = SystemConfig::load_from_flash().unwrap_or_default();

    let mut flight_computer = FlightComputer::new(config);

    flight_computer.initialize();

    defmt::info!("Initialization complete. Entering main loop.");

    loop {
        flight_computer.update();
    }
}