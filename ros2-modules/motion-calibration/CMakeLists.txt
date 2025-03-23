cmake_minimum_required(VERSION 3.8)
project(motion-calibration)

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

enable_testing()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(digital_values_tests tests/digital_values_tests.cpp
                                       src/digital_values.cpp)

  target_include_directories(digital_values_tests PUBLIC
                             $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                             $<INSTALL_INTERFACE:include>
  )
endif()

ament_package()
