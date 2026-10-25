find_package(livox_ros_driver2 REQUIRED)
find_package(std_srvs REQUIRED)
add_executable(${PROJECT_NAME}
  main_ada.cpp
  common_utility.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${CERES_INCLUDE_DIRS}
    ${YAML_INCLUDE_DIRS}
)

# Link against ROS2/ament dependencies so headers like nav_msgs and pcl_conversions are available
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  sensor_msgs
  pcl_conversions
  tf2_ros
  message_filters
  tf2_geometry_msgs
  livox_ros_driver2
  std_srvs
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}.common
  ${PROJECT_NAME}.pre
  ${PROJECT_NAME}.lio
  ${PROJECT_NAME}.algo
  scantext_mapping_core
  scantext_core
  ${third_party_libs}
  glog::glog
  gflags
)

# Install the executable
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ../../launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY ../../config/
  DESTINATION share/${PROJECT_NAME}/config
)