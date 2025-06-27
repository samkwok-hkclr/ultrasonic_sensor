cmake_minimum_required(VERSION 3.8)
project(ultrasonic_sensor)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(canopen_interfaces REQUIRED)

add_executable(ultrasonic_sensor_node 
  src/main.cpp
  src/ultrasonic_sensor_node.cpp)
target_include_directories(ultrasonic_sensor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ultrasonic_sensor_node 
  rclcpp 
  std_msgs
  sensor_msgs
  canopen_interfaces
)
target_compile_features(ultrasonic_sensor_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ultrasonic_sensor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
