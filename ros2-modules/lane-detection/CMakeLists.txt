cmake_minimum_required(VERSION 3.8)
project(lane-detection)

option(ENABLE_TESTS "Enable tests" OFF)
option(ENABLE_COMPILER_OPTIONS "Enable advanced complier options/warnings" OFF)
option(ENABLE_LATEST_CPP_STANDARD "Enable latest c++ standard" OFF)
option(ENABLE_RASPBERY_BUILD "Enable respberry build" OFF)

message(STATUS "Enable tests - " ${ENABLE_TESTS})
message(STATUS "Enable advanced complier options/warnings - " ${ENABLE_COMPILER_OPTIONS})
message(STATUS "Enable latest C++ standard - " ${ENABLE_LATEST_CPP_STANDARD})
message(STATUS "Enable respberry build - " ${ENABLE_RASPBERY_BUILD})

if (ENABLE_COMPILER_OPTIONS)
    add_compile_options(-Wall -Wextra -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Wunused -Woverloaded-virtual -Wpedantic -Wconversion -Wsign-conversion -Wmisleading-indentation -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wnull-dereference -Wuseless-cast -Wdouble-promotion -Wformat=2 -Wimplicit-fallthrough)
endif()

if (ENABLE_RASPBERY_BUILD)
  add_compile_options(-DENABLE_RASPBERY_BUILD)
endif()

if (ENABLE_LATEST_CPP_STANDARD)
    set(CMAKE_CXX_STANDARD 23)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common REQUIRED)
find_package(interfaces REQUIRED)
find_package(OpenCV REQUIRED )

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_executable(${PROJECT_NAME} src/main.cpp
                               src/ros_module.cpp
                               src/lane.cpp
                               src/lane_detection_module.cpp
)
ament_target_dependencies(${PROJECT_NAME} common)

ament_export_include_directories(include)

if (ENABLE_RASPBERY_BUILD)
	target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} -llccv)
else()
	target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
endif()
  
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(TARGETS
        ${PROJECT_NAME}
        DESTINATION
        lib/${PROJECT_NAME}
)

if(ENABLE_TESTS)
  enable_testing()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_lane_tests tests/lane_tests.cpp
                                             src/lane.cpp)

  target_include_directories(${PROJECT_NAME}_lane_tests PUBLIC
                             $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                             $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_lane_tests ${OpenCV_LIBS})

  ament_add_gtest(${PROJECT_NAME}_module_tests tests/lane_detection_module_tests.cpp
                                                     src/lane_detection_module.cpp
                                                     src/lane.cpp)
  target_include_directories(${PROJECT_NAME}_module_tests PUBLIC
                             $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                             $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_module_tests ${OpenCV_LIBS})

endif()

ament_package()

add_executable(${PROJECT_NAME}-app
               src/lane_detection_app.cpp
               src/lane.cpp
               src/lane_detection_module.cpp
)
if (ENABLE_RASPBERY_BUILD)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME}-app ${OpenCV_LIBS} -llccv)
else()
  TARGET_LINK_LIBRARIES(${PROJECT_NAME}-app ${OpenCV_LIBS})
endif()
