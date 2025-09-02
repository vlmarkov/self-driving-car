cmake_minimum_required(VERSION 3.8)
project(slam)

option(ENABLE_TESTS "Enable tests" ON)
option(ENABLE_COMPILER_OPTIONS "Enable advanced complier options/warnings" ON)
option(ENABLE_LATEST_CPP_STANDARD "Enable latest c++ standard" ON)

message(STATUS "Enable tests - " ${ENABLE_TESTS})
message(STATUS "Enable advanced complier options/warnings - " ${ENABLE_COMPILER_OPTIONS})
message(STATUS "Enable latest C++ standard - " ${ENABLE_LATEST_CPP_STANDARD})

if (ENABLE_COMPILER_OPTIONS)
    add_compile_options(-Wall -Wextra -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Wunused -Woverloaded-virtual -Wpedantic -Wconversion -Wsign-conversion -Wmisleading-indentation -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wnull-dereference -Wuseless-cast -Wdouble-promotion -Wformat=2 -Wimplicit-fallthrough)
endif()

if (ENABLE_LATEST_CPP_STANDARD)
    set(CMAKE_CXX_STANDARD 23)
endif()

add_subdirectory(src)

if (ENABLE_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
