# firmware_sil/CMakeLists.txt
#
# Builds the `fh_sim` pybind11 module from the EXACT firmware control sources
# (pulled by path from code/firmware/src — no copies) compiled against the host
# shims in hal/. The result is fh_sim.<ext> in the build dir; sil_bridge.py loads
# it and drives PyBullet with the servo angles the real firmware computes.
#
# Build:
#   cd code/simulation/firmware_sil
#   cmake -S . -B build -DPython_EXECUTABLE=$(which python)
#   cmake --build build
cmake_minimum_required(VERSION 3.15)
project(fh_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use the active interpreter (the conda env) so pybind11 + the .so match it.
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# code/simulation/firmware_sil/ -> code/firmware/src
set(FW "${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/src")

pybind11_add_module(fh_sim
    bindings.cpp
    "${FW}/nervous_system/spinal_cord.cpp"
    "${FW}/nervous_system/servo.cpp"
    "${FW}/nervous_system/leg.cpp"
    "${FW}/nervous_system/motion_math.cpp"
    "${FW}/nervous_system/kinematics.cpp"
)

# hal/ FIRST so its Arduino.h / Wire.h / Adafruit_PWMServoDriver.h shadow any
# real Arduino headers; then the firmware tree for its own headers.
target_include_directories(fh_sim PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/hal"
    "${FW}"
    "${FW}/nervous_system"
    "${FW}/shared"
)
