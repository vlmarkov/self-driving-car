cmake_minimum_required(VERSION 3.8)
project(arduino-code)

message(STATUS "Enable tests - " ${BUILD_TESTING})
message(STATUS "Advanced complier options - " ${ADVANCED_COMPILER_OPTIONS})
message(STATUS "Latest cpp standard - " ${LATEST_CPP_STANDARD})

if (ADVANCED_COMPILER_OPTIONS)
    add_compile_options(-Wall -Wextra -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Wunused -Woverloaded-virtual -Wpedantic -Wconversion -Wsign-conversion -Wmisleading-indentation -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wnull-dereference -Wuseless-cast -Wdouble-promotion -Wformat=2 -Wimplicit-fallthrough)
endif()

if (LATEST_CPP_STANDARD)
    set(CMAKE_CXX_STANDARD 23)
endif()

add_subdirectory(main)
add_subdirectory(tests)
