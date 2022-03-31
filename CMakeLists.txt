cmake_minimum_required(VERSION 3.5)
project(ydlidar_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(SDK_PATH "./sdk/")
FILE(GLOB SDK_SRC
  "${SDK_PATH}/src/impl/unix/*.cpp"
  "${SDK_PATH}/src/*.cpp"
  "${SDK_PATH}/src/*.c"
)
include_directories(
  include
  include/ydlidar_ros2
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/sdk/include
  ${PROJECT_SOURCE_DIR}/sdk/src
)
set(THIS_PACKAGE_LIBS 
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  sensor_msgs
)


add_executable(ydlidar 
  src/ydlidar_node.cpp
  src/ydlidar.cpp
  ${SDK_SRC}
)
target_link_libraries(ydlidar
  ${THIS_PACKAGE_LIBS}
)
target_include_directories(ydlidar
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/src>
  $<INSTALL_INTERFACE:include>  
  ${THIS_PACKAGE_INCLUDE_DIRS}
)
ament_target_dependencies(ydlidar
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(
  TARGETS ydlidar
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY startup sdk launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()