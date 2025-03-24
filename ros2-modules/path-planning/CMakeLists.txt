cmake_minimum_required(VERSION 3.8)
project(path-planning)

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
find_package(common REQUIRED)
find_package(interfaces REQUIRED)

include_directories(include)

add_executable(path_planning src/main.cpp
                             src/path_planning.cpp
)

ament_target_dependencies(path_planning common)

ament_export_include_directories(include)

install(
  DIRECTORY include/path-planning
  DESTINATION include
)

install(TARGETS
        path_planning
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
