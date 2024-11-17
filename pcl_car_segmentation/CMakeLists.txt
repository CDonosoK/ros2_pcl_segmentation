cmake_minimum_required(VERSION 3.8)
project(pcl_car_segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(car_segmentation_by_clusters src/car_segmentation_by_clusters.cpp)
add_executable(cloud_pre_processing src/cloud_pre_processing.cpp)

ament_target_dependencies(
  car_segmentation_by_clusters
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  visualization_msgs
)

ament_target_dependencies(
  cloud_pre_processing
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  visualization_msgs
)

target_link_libraries(
  car_segmentation_by_clusters
  ${PCL_LIBRARIES}
)

target_link_libraries(
  cloud_pre_processing
  ${PCL_LIBRARIES}
)

install(TARGETS
car_segmentation_by_clusters
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  cloud_pre_processing
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
