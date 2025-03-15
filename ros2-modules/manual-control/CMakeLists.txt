cmake_minimum_required(VERSION 3.8)
project(manual-control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common REQUIRED)
find_package(interfaces REQUIRED)

include_directories(include)

add_executable(manual_control src/main.cpp
                              src/manual_control.cpp
                              src/network/socket.cpp
                              src/network/tcp_server.cpp
)

ament_target_dependencies(manual_control common)

ament_export_include_directories(include)

install(
  DIRECTORY include/manual-control
  DESTINATION include
)

install(TARGETS
        manual_control
        DESTINATION
        lib/${PROJECT_NAME}
)

enable_testing()

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(tcp_client_tests tests/tcp_client_tests.cpp
                                   src/network/tcp_client.cpp)

  target_include_directories(tcp_client_tests PUBLIC
                             $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                             $<INSTALL_INTERFACE:include>
  )

  ament_add_gmock(tcp_server_tests tests/tcp_server_tests.cpp
                                   src/network/tcp_server.cpp)

  target_include_directories(tcp_server_tests PUBLIC
                             $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                             $<INSTALL_INTERFACE:include>
  )

endif()

ament_package()

add_executable(${PROJECT_NAME}_app
               src/manual_control_app.cpp
               src/network/socket.cpp
               src/network/tcp_client.cpp
)
