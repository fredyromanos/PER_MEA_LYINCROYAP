cmake_minimum_required(VERSION 3.10)
project(drone_test CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Host-native build of the firmware LOGIC layers — no ESP32, no Arduino core.
# stubs/Arduino.h satisfies the few `#include <Arduino.h>` in the headers under test.
include_directories(
    ${CMAKE_SOURCE_DIR}/stubs        # Arduino.h stub (searched first)
    ${CMAKE_SOURCE_DIR}/../main/src  # current firmware layout (post src/ reorg)
)

enable_testing()

# --- Control-logic suite (ModeManager + unified ManualController) -------------
add_executable(test_runner
    test_runner.cpp
    ../main/src/control/ManualController.cpp
    ../main/src/control/ModeManager.cpp
)
add_test(NAME control_logic COMMAND test_runner)

# --- Navigation suite (Navigator + MissionManager + navigation.h) ------------
# navigation.h is header-only; Navigator/MissionManager have .cpp bodies.
add_executable(test_nav
    test_nav.cpp
    ../main/src/navigation/Navigator.cpp
    ../main/src/navigation/MissionManager.cpp
)
add_test(NAME navigation COMMAND test_nav)

# --- HW-input seam suite (BatteryAdc divider math via faked ADC) --------------
add_executable(test_hw
    test_hw.cpp
    ../main/src/drivers/BatteryAdc.cpp
)
add_test(NAME hw_seam COMMAND test_hw)

# --- GPS-only heading suite (HeadingGate + AutoController + heartbeat budget) --
add_executable(test_gps_logic
    test_gps_logic.cpp
    ../main/src/control/HeadingGate.cpp
    ../main/src/control/AutoController.cpp
    ../main/src/navigation/Navigator.cpp
)
add_test(NAME gps_logic COMMAND test_gps_logic)
