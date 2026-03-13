cmake_minimum_required(VERSION 3.10)
project(motor_controller VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. SET PATHS
set(SPARKCAN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/sparkcan")
set(PYBIND_INC_ENV "/home/luna01/PantherLunabotics2025-2026/lunaenv/lib/python3.10/site-packages/pybind11/include")
set(PYTHON_INC_ENV "/usr/include/python3.10")

# 2. DEFINE THE MODULE
# Create Python module from MotorControler.cpp
add_library(motor_controller MODULE
    MotorController.cpp
    ${SPARKCAN_DIR}/src/SparkBase.cpp
    ${SPARKCAN_DIR}/src/SparkMax.cpp
    ${SPARKCAN_DIR}/src/SparkFlex.cpp
)

# 3. INCLUDE DIRECTORIES
target_include_directories(motor_controller PRIVATE
    ${PYBIND_INC_ENV}
    ${PYTHON_INC_ENV}
    ${SPARKCAN_DIR}/include
)

# 5. LINK SYSTEM DEPENDENCIES
# The Jetson needs pthread to handle the CAN heartbeats
target_link_libraries(motor_controller PRIVATE pthread)

# 6. SET OUTPUT PROPERTIES
# Remove the 'lib' prefix and set the output name
set_target_properties(motor_controller PROPERTIES
    PREFIX ""
    OUTPUT_NAME "motor_controller"
)
