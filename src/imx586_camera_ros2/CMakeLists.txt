cmake_minimum_required(VERSION 3.16)
project(imx586_camera_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs aruco)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0)

add_executable(imx586_dual_camera_node src/imx586_dual_camera_node.cpp)
ament_target_dependencies(imx586_dual_camera_node rclcpp sensor_msgs std_msgs)
target_link_libraries(imx586_dual_camera_node PkgConfig::GSTREAMER)

add_executable(fast_calib2_monitor_node src/fast_calib2_monitor_node.cpp)
ament_target_dependencies(fast_calib2_monitor_node rclcpp sensor_msgs)
target_include_directories(fast_calib2_monitor_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fast_calib2_monitor_node ${OpenCV_LIBRARIES})

install(TARGETS imx586_dual_camera_node fast_calib2_monitor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
