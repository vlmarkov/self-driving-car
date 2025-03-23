cmake_minimum_required(VERSION 3.8)
project(common)

# let the compiler search for headers in the include folder
include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

add_library(common src/base_pub_sub_node.cpp)

ament_target_dependencies(common rclcpp interfaces)
ament_export_targets(common HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/common
  DESTINATION include
)

install(
  TARGETS common
  EXPORT common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()

