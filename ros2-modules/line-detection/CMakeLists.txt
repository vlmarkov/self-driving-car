cmake_minimum_required(VERSION 3.8)
project(line-detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common REQUIRED)
find_package(interfaces REQUIRED)
find_package(OpenCV REQUIRED )

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_executable(line_detection src/main.cpp
                              src/line_detection.cpp
)
ament_target_dependencies(line_detection common)

ament_export_include_directories(include)

install(
  DIRECTORY include/line-detection
  DESTINATION include
)

install(
  TARGETS line_detection
  EXPORT line_detection
  INCLUDES DESTINATION include
)

install(TARGETS
        line_detection
        DESTINATION
        lib/${PROJECT_NAME}
)

enable_testing()

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_lane_tests tests/lane_tests.cpp
                                             src/lane.cpp)

  target_include_directories(${PROJECT_NAME}_lane_tests PUBLIC
                             $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                             $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_lane_tests ${OpenCV_LIBS})

  ament_add_gtest(${PROJECT_NAME}_line_detection_module_tests tests/line_detection_module_tests.cpp
                                                              src/line_detection_module.cpp
                                                              src/lane.cpp)
  target_include_directories(${PROJECT_NAME}_line_detection_module_tests PUBLIC
                             $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                             $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_line_detection_module_tests ${OpenCV_LIBS})

endif()

ament_package()

add_executable(line_detection_shell_app
               src/shell_app.cpp
               src/lane.cpp
               src/line_detection_module.cpp
)
TARGET_LINK_LIBRARIES(line_detection_shell_app ${OpenCV_LIBS})
