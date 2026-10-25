add_library(${PROJECT_NAME}.pre
        cloud_convert/cloud_convert2.cc
        )

target_link_libraries(${PROJECT_NAME}.pre
        ${third_party_libs}
        )

# Make this target depend on ROS2/ament packages so include dirs and link info
# from rclcpp and pcl_conversions are available (pcl_conversions headers include rclcpp)
ament_target_dependencies(${PROJECT_NAME}.pre
  rclcpp
  pcl_conversions
  message_filters
  livox_ros_driver2

)


# 导出库以便其他包可以使用
ament_export_targets(${PROJECT_NAME}.pre HAS_LIBRARY_TARGET)

# 安装库文件
install(
  TARGETS ${PROJECT_NAME}.pre
  EXPORT ${PROJECT_NAME}.pre
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)