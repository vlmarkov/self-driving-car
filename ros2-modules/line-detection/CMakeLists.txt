cmake_minimum_required(VERSION 3.8)
project(line-detection)

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

add_executable(line_detection src/main.cpp
                              src/line_detection.cpp
)
ament_target_dependencies(line_detection common)

ament_export_include_directories(include)

install(
  DIRECTORY include/line-detection
  DESTINATION include
)

install(
  TARGETS line_detection
  EXPORT line_detection
  INCLUDES DESTINATION include
)

install(TARGETS
        line_detection
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
