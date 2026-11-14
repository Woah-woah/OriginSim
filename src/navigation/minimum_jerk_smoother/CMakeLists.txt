cmake_minimum_required(VERSION 3.18)
project(minimum_jerk_smoother LANGUAGES C CXX)

# Eigen 稠密矩阵和采样循环默认启用优化；调用者仍可显式选择 Debug。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(map_server REQUIRED)
find_package(topology_global_planner REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 本地求解器以静态库链接到平滑器，不安装第三方目标和头文件。
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "" FORCE)
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_INTERRUPT OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_DERIVATIVES OFF CACHE BOOL "" FORCE)
set(OSQP_CODEGEN OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/osqp EXCLUDE_FROM_ALL)

# 仅编译 Eigen 封装库，避免引入上游独立项目的安装和测试规则。
set(osqp_eigen_dir "${CMAKE_CURRENT_SOURCE_DIR}/third_party/osqp-eigen")
add_library(jerk_osqp_eigen STATIC
  ${osqp_eigen_dir}/src/Data.cpp
  ${osqp_eigen_dir}/src/Settings.cpp
  ${osqp_eigen_dir}/src/Solver.cpp
  ${osqp_eigen_dir}/src/Debug.cpp
)
target_compile_features(jerk_osqp_eigen PUBLIC cxx_std_14)
target_include_directories(jerk_osqp_eigen SYSTEM PUBLIC ${osqp_eigen_dir}/include)
target_compile_definitions(jerk_osqp_eigen PUBLIC
  OSQP_EIGEN_OSQP_IS_V1
  OSQP_EIGEN_OSQP_IS_V1_FINAL
)
target_link_libraries(jerk_osqp_eigen PUBLIC osqpstatic Eigen3::Eigen)

# 平滑器公开接口仅导出 ROS 依赖，求解器保留为内部实现。
add_library(${PROJECT_NAME} SHARED
  src/smoother.cpp
  src/tdt/minimumSnap.cpp
  src/tdt/sfcSquare.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PRIVATE jerk_osqp_eigen Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  map_server
  topology_global_planner
  nav_msgs
  geometry_msgs
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES LICENSE src/tdt/UPSTREAM DESTINATION share/${PROJECT_NAME})
install(FILES third_party/osqp/LICENSE
  DESTINATION share/${PROJECT_NAME}
  RENAME OSQP-LICENSE
)
install(FILES ${osqp_eigen_dir}/LICENSE
  DESTINATION share/${PROJECT_NAME}
  RENAME OsqpEigen-LICENSE
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(map_server topology_global_planner nav_msgs geometry_msgs)
ament_package()
