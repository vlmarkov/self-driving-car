cmake_minimum_required(VERSION 3.8)
project(main-pipeline)

message(STATUS "Enable tests - " ${BUILD_TESTING})
message(STATUS "Advanced complier options - " ${ADVANCED_COMPILER_OPTIONS})
message(STATUS "Latest cpp standard - " ${LATEST_CPP_STANDARD})

if (ADVANCED_COMPILER_OPTIONS)
    add_compile_options(-Wall -Wextra -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Wunused -Woverloaded-virtual -Wpedantic -Wconversion -Wsign-conversion -Wmisleading-indentation -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wnull-dereference -Wuseless-cast -Wdouble-promotion -Wformat=2 -Wimplicit-fallthrough)
endif()

if (LATEST_CPP_STANDARD)
    set(CMAKE_CXX_STANDARD 23)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(interfaces REQUIRED)
find_package(common REQUIRED)
find_package(manual-control REQUIRED)
find_package(path-planning REQUIRED)
find_package(obstacle-avoidance REQUIRED)
find_package(lane-detection REQUIRED)
find_package(motion-calibration REQUIRED)

include_directories(include)

add_executable(main_pipeline src/main.cpp
                             src/ros_module.cpp
)

ament_target_dependencies(main_pipeline
                          rclcpp
                          common
                          manual-control
                          path-planning
                          obstacle-avoidance
                          lane-detection
                          motion-calibration)

install(TARGETS
        main_pipeline
        DESTINATION
        lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  #find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_package()
