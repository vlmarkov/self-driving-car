cmake_minimum_required(VERSION 3.8)

enable_testing()

find_package(GTest REQUIRED)

add_executable(motor_commands_tests
               motor_commands_tests.cpp
)

target_link_libraries(motor_commands_tests
                      motor-commands-lib
                      GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(motor_commands_tests)
