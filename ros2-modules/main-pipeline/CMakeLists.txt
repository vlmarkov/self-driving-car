cmake_minimum_required(VERSION 3.8)
project(main-pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)

find_package(common REQUIRED)
find_package(manual-control REQUIRED)
find_package(path-planning REQUIRED)
find_package(obstacle-avoidance REQUIRED)
find_package(line-detection REQUIRED)
find_package(motion-calibration REQUIRED)

include_directories(include)

add_executable(main_pipeline src/main.cpp
                             src/main_pipeline.cpp
)

ament_target_dependencies(main_pipeline
                          rclcpp
                          std_msgs
                          common
                          manual-control
                          path-planning
                          obstacle-avoidance
                          line-detection
                          motion-calibration)

install(TARGETS
        main_pipeline
        DESTINATION
        lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
