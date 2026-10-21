cmake_minimum_required(VERSION 3.16)
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
find_package(OsqpEigen QUIET)

if(OsqpEigen_FOUND)
  set(jerk_solver OsqpEigen::OsqpEigen)
else()
  # 与东大参考工程固定相同版本；已有系统依赖时优先复用。
  include(FetchContent)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
  set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "" FORCE)
  set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
  set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
  set(OSQP_ENABLE_INTERRUPT OFF CACHE BOOL "" FORCE)
  set(OSQP_ENABLE_DERIVATIVES OFF CACHE BOOL "" FORCE)
  set(OSQP_CODEGEN OFF CACHE BOOL "" FORCE)
  fetchcontent_declare(jerk_osqp
    GIT_REPOSITORY https://github.com/osqp/osqp.git
    GIT_TAG v1.0.0
    GIT_SHALLOW TRUE
    GIT_SUBMODULES "")
  fetchcontent_makeavailable(jerk_osqp)
  fetchcontent_declare(jerk_eigen
    GIT_REPOSITORY https://github.com/robotology/osqp-eigen.git
    GIT_TAG v0.11.0
    GIT_SHALLOW TRUE
    GIT_SUBMODULES "")
  fetchcontent_getproperties(jerk_eigen)
  if(NOT jerk_eigen_POPULATED)
    fetchcontent_populate(jerk_eigen)
  endif()
  # 校验固定 tag 的提交，既支持浅克隆，也防止上游 tag 漂移。
  find_package(Git REQUIRED)
  execute_process(COMMAND ${GIT_EXECUTABLE} -C ${jerk_osqp_SOURCE_DIR} rev-parse HEAD
    OUTPUT_VARIABLE jerk_osqp_revision OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND ${GIT_EXECUTABLE} -C ${jerk_eigen_SOURCE_DIR} rev-parse HEAD
    OUTPUT_VARIABLE jerk_eigen_revision OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(NOT jerk_osqp_revision STREQUAL "236713ce9a56c182ac3230d52108f952afce1523" OR
    NOT jerk_eigen_revision STREQUAL "b79df5019192550215db28874ec6efa405671918")
    message(FATAL_ERROR "OSQP/OsqpEigen dependency revision mismatch")
  endif()
  # 仅编译封装库，避免第三方安装规则及测试污染 ROS 包导出。
  add_library(jerk_osqp_eigen STATIC
    ${jerk_eigen_SOURCE_DIR}/src/Data.cpp
    ${jerk_eigen_SOURCE_DIR}/src/Settings.cpp
    ${jerk_eigen_SOURCE_DIR}/src/Solver.cpp
    ${jerk_eigen_SOURCE_DIR}/src/Debug.cpp)
  target_include_directories(jerk_osqp_eigen PUBLIC ${jerk_eigen_SOURCE_DIR}/include)
  target_compile_definitions(jerk_osqp_eigen PUBLIC
    OSQP_EIGEN_OSQP_IS_V1 OSQP_EIGEN_OSQP_IS_V1_FINAL)
  target_link_libraries(jerk_osqp_eigen PUBLIC osqpstatic Eigen3::Eigen)
  set(jerk_solver jerk_osqp_eigen)
  install(FILES ${jerk_eigen_SOURCE_DIR}/LICENSE
    DESTINATION share/${PROJECT_NAME} RENAME OsqpEigen-LICENSE)
  install(FILES ${jerk_osqp_SOURCE_DIR}/LICENSE
    DESTINATION share/${PROJECT_NAME} RENAME OSQP-LICENSE)
endif()

add_library(${PROJECT_NAME} SHARED src/smoother.cpp
  src/tdt/minimumSnap.cpp src/tdt/sfcSquare.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PRIVATE ${jerk_solver} Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} PUBLIC map_server topology_global_planner nav_msgs geometry_msgs)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES LICENSE src/tdt/UPSTREAM DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(map_server topology_global_planner nav_msgs geometry_msgs)
ament_package()
