# CMakeLists.txt for CRS Car Firmware
# For a reference to the ESP-IDF build system, visit:
# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html
cmake_minimum_required(VERSION 3.5)
set(EXTRA_COMPONENT_DIRS
    "./src/communication"
    "./src/configuration"
    "./src/control"
    "./src/hardware"
    "./src/interfaces"
    "./src/main"
    "./src/rtos"
    "./src/state"
    "./src/util"
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(car-firmware)
