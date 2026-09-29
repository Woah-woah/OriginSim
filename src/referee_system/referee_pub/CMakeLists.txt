cmake_minimum_required(VERSION 3.8 FATAL_ERROR) 
project(referee_pub)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(referee_msg REQUIRED)

# Find Qt5
find_package (Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
)


# Find the Gazebo gui library
find_package(gz-gui8 REQUIRED)
find_package(gz-common5 REQUIRED)


add_executable(referee_pub src/referee_publisher.cpp)
ament_target_dependencies(referee_pub 
  rclcpp
  std_msgs
  referee_msg
)

include_directories(include)
include_directories(SYSTEM
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
  ${Qt5QuickControls2_INCLUDE_DIRS}
)

target_link_libraries(referee_pub
gz-gui8::gz-gui8
${Qt5Core_LIBRARIES}
${Qt5Qml_LIBRARIES}
${Qt5Quick_LIBRARIES}
${Qt5QuickControls2_LIBRARIES}
)



install(TARGETS
referee_pub
DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
        scripts/referee_sub.py
    DESTINATION lib/${PROJECT_NAME}
)



# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
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
