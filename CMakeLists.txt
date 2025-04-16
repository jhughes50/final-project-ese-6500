cmake_minimum_required(VERSION 3.8)
project(localization)
#TODO better project name

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

set(node_plugins "")

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)

# Libraries  
# Make sure to add your source files
add_library(localization_component SHARED
  src/imu_buffer.cpp
  src/factor_manager.cpp
)

target_link_libraries(localization_component PUBLIC
  gtsam
)

# build utm test
add_executable(utm_test test/test_ll_to_utm.cpp)
target_include_directories(utm_test PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include/
)

