cmake_minimum_required(VERSION 3.8)
project(common)

message(STATUS "Enable tests - " ${BUILD_TESTING})
message(STATUS "Advanced complier options - " ${ADVANCED_COMPILER_OPTIONS})
message(STATUS "Latest cpp standard - " ${LATEST_CPP_STANDARD})

if (ADVANCED_COMPILER_OPTIONS)
    add_compile_options(-Wall -Wextra -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Wunused -Woverloaded-virtual -Wpedantic -Wconversion -Wsign-conversion -Wmisleading-indentation -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wnull-dereference -Wuseless-cast -Wdouble-promotion -Wformat=2 -Wimplicit-fallthrough)
endif()

if (LATEST_CPP_STANDARD)
    set(CMAKE_CXX_STANDARD 23)
endif()

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
  enable_testing()
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

