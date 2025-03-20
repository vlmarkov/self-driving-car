cmake_minimum_required(VERSION 3.8)
project(arduino-code)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++23)
endif()

add_subdirectory(main)
add_subdirectory(tests)
