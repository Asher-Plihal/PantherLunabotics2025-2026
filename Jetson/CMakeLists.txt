#subsystems
cmake_minimum_required(VERSION 3.10)
project(subsystem_drive VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. SET PATHS
set(LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../library/motor_controller/sparkcan")
set(PYBIND_INC_ENV "/home/luna01/Lunabotics/PantherLunabotics2025-2026/lunaenv/lib/python3.10/site-packages/pybind11/include")
set(PYTHON_INC_ENV "/usr/local/python3.10/include/python3.10")

# 2. DEFINE THE MODULE
# We include the sparkcan .cpp files directly in the source list
add_library(motor_lib MODULE 
    motor_control.cpp 
    ${LIB_DIR}/src/SparkBase.cpp
    ${LIB_DIR}/src/SparkMax.cpp
    ${LIB_DIR}/src/SparkFlex.cpp
)

# 3. CONFIGURE THE OUTPUT
set_target_properties(motor_lib PROPERTIES PREFIX "" OUTPUT_NAME "motor_lib")

# 4. INCLUDE DIRECTORIES
target_include_directories(motor_lib PRIVATE 
    ${PYBIND_INC_ENV} 
    ${PYTHON_INC_ENV}
    ${LIB_DIR}/include     # So the compiler can find SparkMax.hpp
)

# 5. LINK SYSTEM DEPENDENCIES
# The Jetson needs pthread to handle the CAN heartbeats
target_link_libraries(motor_lib PRIVATE pthread)