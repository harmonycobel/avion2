cmake_minimum_required(VERSION 3.16)
project(Avion2 VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags for embedded systems
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -O2
    -fno-exceptions
    -fno-rtti
    -ffunction-sections
    -fdata-sections
)

# Add linker flags for size optimization
add_link_options(
    -Wl,--gc-sections
)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Source files
set(SOURCES
    src/core/system_manager.cpp
    src/core/event_loop.cpp
    src/core/state_machine.cpp
    src/sensors/imu.cpp
    src/sensors/gps.cpp
    src/sensors/barometer.cpp
    src/sensors/temperature.cpp
    src/control/flight_controller.cpp
    src/control/guidance.cpp
    src/control/navigation.cpp
    src/control/pid_controller.cpp
    src/telemetry/telemetry_manager.cpp
    src/telemetry/packet_handler.cpp
    src/telemetry/data_logger.cpp
    src/propulsion/engine_controller.cpp
    src/propulsion/throttle_controller.cpp
    src/propulsion/fuel_manager.cpp
    src/safety/abort_system.cpp
    src/safety/fault_detection.cpp
    src/safety/redundancy_manager.cpp
    src/ground_station/command_processor.cpp
    src/ground_station/uplink_handler.cpp
    src/main.cpp
)

# Create executable
add_executable(avion2 ${SOURCES})

# Testing
enable_testing()
add_subdirectory(tests)

# Documentation
find_package(Doxygen)
if(DOXYGEN_FOUND)
    add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)
endif()