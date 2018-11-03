cmake_minimum_required(VERSION 3.5)

# Include additional local components
set(EXTRA_COMPONENT_DIRS "./components/i2c_devices"
        "./components/i2c_devices/sensors")

# Include IDF
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define project
project(ucdfs-telemetry)
