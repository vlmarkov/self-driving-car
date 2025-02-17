cmake_minimum_required(VERSION 3.8)
project(obstacle-avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common REQUIRED)
find_package(interfaces REQUIRED)

include_directories(include)

add_executable(obstacle_avoidance src/main.cpp
                                  src/obstacle_avoidance.cpp 
)

ament_target_dependencies(obstacle_avoidance common)

ament_export_include_directories(include)

install(
  DIRECTORY include/obstacle-avoidance
  DESTINATION include
)

install(TARGETS
        obstacle_avoidance
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
