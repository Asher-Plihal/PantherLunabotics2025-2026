cmake_minimum_required(VERSION 3.10)
project(ecbf_unicycle)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(osqp REQUIRED)

# Add executable
add_executable(ecbf_unicycle ecbf_unicycle.cpp)

# Set output directory to parent folder
set_target_properties(ecbf_unicycle PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}"
)

# Link libraries
target_link_libraries(ecbf_unicycle 
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
    osqp::osqp
)

# Include directories (usually handled by find_package, but just in case)
target_include_directories(ecbf_unicycle PRIVATE 
    ${EIGEN3_INCLUDE_DIRS}
)