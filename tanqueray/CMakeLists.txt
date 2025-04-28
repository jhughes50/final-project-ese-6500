cmake_minimum_required(VERSION 3.8)
project(tanqueray)
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
find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
  ${yaml-cpp_INCLUDE_DIRS}
)

# Libraries  
# Make sure to add your source files
add_library(${PROJECT_NAME} SHARED
  src/imu_buffer.cpp
  src/factor_manager.cpp
  src/params.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    gtsam
    yaml-cpp
    tbb
)

enable_testing()
# build utm test
add_executable(utm_test test/test_ll_to_utm.cpp)
target_link_libraries(utm_test
  GTest::GTest
  GTest::Main
  ${PROJECT_NAME}
)

#build params test
add_executable(params_test test/test_params.cpp)
target_link_libraries(params_test
  GTest::GTest
  GTest::Main
  ${PROJECT_NAME}
)

include(GoogleTest)
gtest_discover_tests(utm_test)
gtest_discover_tests(params_test)
