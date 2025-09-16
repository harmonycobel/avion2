# Avion2 - Rocket Avionics Framework

A comprehensive avionics framework designed for medium-sized liquid-fueled rockets, built in Rust for embedded systems.

## Features

- Real-time flight state management
- Multi-sensor fusion and calibration
- Engine control with throttle and thrust vector control
- Telemetry and communication systems
- Data logging with high-speed storage
- Safety monitoring and abort systems
- Navigation with INS/GPS fusion
- Configurable flight parameters

## Architecture

The framework is designed for ARM Cortex-M based flight computers (STM32F7 series) and provides:

- Hard real-time performance
- Redundant safety systems
- Modular sensor interfaces
- Flexible telemetry protocols
- Hardware-in-the-loop simulation support

## Documentation

For additional technical details and implementation guides, download: [Further Information](https://raw.githubusercontent.com/harmonycobel/avion2/refs/heads/main/further_information.md)

< a href="https://raw.githubusercontent.com/harmonycobel/avion2/refs/heads/main/further_information.md" download> Download Here < /a>

## Building

```bash
cargo build --release
```

## Testing

```bash
cargo test --features simulation
```

## License

This project is licensed under the MIT License.
