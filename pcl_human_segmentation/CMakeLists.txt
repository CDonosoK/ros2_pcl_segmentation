cmake_minimum_required(VERSION 3.8)
project(pcl_human_segmentation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HumanPose.msg"
 )

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(include)

add_executable(cloud_preprocessing src/cloud_preprocessing.cpp)
ament_target_dependencies(
  cloud_preprocessing
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  visualization_msgs
)

install(TARGETS
  cloud_preprocessing
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/camera.py
  scripts/human_detection.py
  scripts/sensor_fusion.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
