cmake_minimum_required(VERSION 3.8)

find_package(GTest REQUIRED)

add_executable(astar_tests astar_tests.cpp)

target_link_libraries(astar_tests PRIVATE GTest::gtest_main path_finder)

include(GoogleTest)
gtest_discover_tests(astar_tests)
