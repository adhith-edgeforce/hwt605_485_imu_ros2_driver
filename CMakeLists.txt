cmake_minimum_required(VERSION 3.8)
project(hwt605_driver_fast)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

add_executable(hwt605_node src/hwt605_node.cpp)
ament_target_dependencies(hwt605_node rclcpp sensor_msgs)
target_link_libraries(hwt605_node Threads::Threads)

install(TARGETS hwt605_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
