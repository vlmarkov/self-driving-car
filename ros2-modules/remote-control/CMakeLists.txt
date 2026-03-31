cmake_minimum_required(VERSION 3.8)
project(remote-control)

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
find_package(std_msgs REQUIRED)
find_package(common REQUIRED)
find_package(interfaces REQUIRED)

find_package(nlohmann_json 3.11.3 REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME} src/main.cpp
                               src/remote-control/commands.cpp
                               src/remote-control/server.cpp
                               src/network/socket.cpp
                               src/network/tcp_server.cpp
)

ament_target_dependencies(${PROJECT_NAME} common)

ament_export_include_directories(include)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(TARGETS
        ${PROJECT_NAME}
        DESTINATION
        lib/${PROJECT_NAME}
)

if(ENABLE_TESTS)
  enable_testing()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(tcp_client_tests tests/tcp_client_tests.cpp
                                   src/remote-control/commands.cpp
                                   src/network/tcp_client.cpp)

  ament_add_gmock(tcp_server_tests tests/tcp_server_tests.cpp
                                   src/remote-control/commands.cpp
                                   src/network/tcp_server.cpp)

  ament_add_gmock(client_tests tests/client_tests.cpp
                                              src/remote-control/client.cpp
                                              src/remote-control/commands.cpp
                                              src/network/tcp_client.cpp)

  ament_add_gmock(server_tests tests/server_tests.cpp
                                              src/remote-control/server.cpp
                                              src/remote-control/commands.cpp
                                              src/network/tcp_server.cpp)
  ament_target_dependencies(server_tests common)

endif()

ament_package()

add_executable(${PROJECT_NAME}-app
               src/remote-control/app.cpp
               src/remote-control/client.cpp
               src/remote-control/commands.cpp
               src/network/socket.cpp
               src/network/tcp_client.cpp
)
