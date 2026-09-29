cmake_minimum_required(VERSION 3.8)
project(region_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetRegion.srv"
  DEPENDENCIES geometry_msgs
)

ament_auto_add_executable(region_behavior_node
  src/region_behavior_node.cpp
)

target_link_libraries(region_behavior_node
  yaml-cpp
)

rosidl_target_interfaces(region_behavior_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS region_behavior_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY include/ DESTINATION include)

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
