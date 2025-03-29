cmake_minimum_required(VERSION 3.8)
project(motion-calibration)

option(ENABLE_TESTS "Enable tests" OFF)
option(ENABLE_COMPILER_OPTIONS "Enable advanced complier options/warnings" OFF)
option(ENABLE_LATEST_CPP_STANDARD "Enable latest c++ standard" OFF)

message(STATUS "Enable tests - " ${ENABLE_TESTS})
message(STATUS "Enable advanced complier options/warnings - " ${ENABLE_COMPILER_OPTIONS})
message(STATUS "Enable latest C++ standard - " ${ENABLE_LATEST_CPP_STANDARD})

if (ENABLE_COMPILER_OPTIONS)
    add_compile_options(-Wall -Wextra -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Wunused -Woverloaded-virtual -Wpedantic -Wconversion -Wsign-conversion -Wmisleading-indentation -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wnull-dereference -Wuseless-cast -Wdouble-promotion -Wformat=2 -Wimplicit-fallthrough)
endif()

if (ENABLE_LATEST_CPP_STANDARD)
    set(CMAKE_CXX_STANDARD 23)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common REQUIRED)
find_package(interfaces REQUIRED)
find_package(fmt)

include_directories(include)

add_executable(motion_calibration src/main.cpp
                                  src/ros_module.cpp
                                  src/digital_values.cpp
)

ament_target_dependencies(motion_calibration common)

target_link_libraries(motion_calibration fmt::fmt)

ament_export_include_directories(include)

install(
  DIRECTORY include/motion-calibration
  DESTINATION include
)

install(TARGETS
        motion_calibration
        DESTINATION
        lib/${PROJECT_NAME}
)

if(ENABLE_TESTS)
  enable_testing()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(digital_values_tests tests/digital_values_tests.cpp
                                       src/digital_values.cpp)

  target_include_directories(digital_values_tests PUBLIC
                             $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                             $<INSTALL_INTERFACE:include>
  )
endif()

ament_package()
