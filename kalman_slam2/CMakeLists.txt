cmake_minimum_required(VERSION 3.8)
project(kalman_slam2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS core io)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

add_executable(laser_filter_node src/laser_filter_node.cpp)
ament_target_dependencies(laser_filter_node rclcpp sensor_msgs)

add_executable(local_maxima_node src/local_maxima_node.cpp)
ament_target_dependencies(local_maxima_node 
  rclcpp 
  sensor_msgs 
  visualization_msgs
  pcl_conversions
)

add_library(pointcloud_concat_node SHARED src/pointcloud_concat_node.cpp)
ament_target_dependencies(pointcloud_concat_node rclcpp rclcpp_components geometry_msgs tf2_ros pcl_ros)
rclcpp_components_register_node(
  pointcloud_concat_node
  PLUGIN "pointcloud_concatenate::PointCloudConcatNode"
  EXECUTABLE pointcloud_concat
)

ament_export_targets(kalman_slam2-targets)
install(TARGETS pointcloud_concat_node 
  EXPORT kalman_slam2-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

target_link_libraries(local_maxima_node 
  ${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS local_maxima_node laser_filter_node 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
