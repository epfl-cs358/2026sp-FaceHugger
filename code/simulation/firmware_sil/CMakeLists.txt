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

# ArduinoJson — the firmware's JSON lib; header-only and host-portable. Used by
# the compiled network.cpp command dispatch. Fetched (network) at configure time.
include(FetchContent)
FetchContent_Declare(
    ArduinoJson
    GIT_REPOSITORY https://github.com/bblanchon/ArduinoJson.git
    GIT_TAG v7.2.1
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(ArduinoJson)

# code/simulation/firmware_sil/ -> code/firmware/src
set(FW "${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/src")

pybind11_add_module(fh_sim
    bindings.cpp
    "${FW}/nervous_system/spinal_cord.cpp"
    "${FW}/nervous_system/servo.cpp"
    "${FW}/nervous_system/leg.cpp"
    "${FW}/nervous_system/motion_math.cpp"
    "${FW}/nervous_system/kinematics.cpp"
    "${FW}/nervous_system/face.cpp"      # OLED eyes; no-op against the hal/ display stubs
    "${FW}/brain/network.cpp"            # the EXACT T: command dispatch
    "${FW}/brain/clip_list_serializer.cpp"
    # brain/sensors.cpp is INTENTIONALLY excluded — the production version talks
    # to a real MPU6050 over Wire. We substitute hal/sensors_sil.cpp instead
    # (same API surface from brain/sensors.h, but state comes from PyBullet via
    # fh_sim::imu_upside_down / pitch_deg / roll_deg, written by the bridge).
    hal/sensors_sil.cpp
)

# hal/ FIRST so its Arduino.h / Wire.h / Adafruit_PWMServoDriver.h / WiFi.h /
# WebSocketsServer.h shadow the real Arduino headers; then the firmware tree.
target_include_directories(fh_sim PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/hal"
    "${FW}"
    "${FW}/nervous_system"
    "${FW}/shared"
    "${FW}/brain"
)
target_link_libraries(fh_sim PRIVATE ArduinoJson)

# Keep ArduinoJson host-only: don't pull its Arduino String/Stream/Print
# integration (our hal/Arduino.h is minimal). std::string support stays on.
target_compile_definitions(fh_sim PRIVATE
    ARDUINOJSON_ENABLE_ARDUINO_STRING=0
    ARDUINOJSON_ENABLE_ARDUINO_STREAM=0
    ARDUINOJSON_ENABLE_ARDUINO_PRINT=0
    ARDUINOJSON_ENABLE_PROGMEM=0
    ARDUINOJSON_ENABLE_STD_STRING=1
)
