cmake_minimum_required(VERSION 3.8)
project(obstacle-avoidance)

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

if(ENABLE_TESTS)
  enable_testing()
  #find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_package()
