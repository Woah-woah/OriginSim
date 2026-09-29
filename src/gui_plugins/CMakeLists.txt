cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gui_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set (CMAKE_AUTOMOC ON)

# Find Qt5
find_package (Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
)

find_package(ament_cmake REQUIRED)

# Find the Gazebo gui library
find_package(gz-gui8 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) 

QT5_ADD_RESOURCES(resources_RCC plugins/referee_ui/referee.qrc)
QT5_ADD_RESOURCES(resources_RCC plugins/race_control/racecontrol.qrc)

include_directories(SYSTEM
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
  ${Qt5QuickControls2_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(RefereeUI SHARED
  plugins/referee_ui/RefereeUI.cc
  ${resources_RCC}
)

target_link_libraries(RefereeUI
  gz-gui8::gz-gui8
  ${Qt5Core_LIBRARIES}
  ${Qt5Qml_LIBRARIES}
  ${Qt5Quick_LIBRARIES}
  ${Qt5QuickControls2_LIBRARIES}
  gz-transport13::gz-transport13
)

install(TARGETS RefereeUI
    DESTINATION plugins
)

add_library(RaceControl SHARED
  plugins/race_control/RaceControl.cc
  ${resources_RCC}
)

target_link_libraries(RaceControl
  gz-gui8::gz-gui8
  ${Qt5Core_LIBRARIES}
  ${Qt5Qml_LIBRARIES}
  ${Qt5Quick_LIBRARIES}
  ${Qt5QuickControls2_LIBRARIES}
  gz-transport13::gz-transport13
  yaml-cpp 
)

ament_target_dependencies(RaceControl
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS RaceControl
    DESTINATION plugins
)

#environment
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

ament_package()
