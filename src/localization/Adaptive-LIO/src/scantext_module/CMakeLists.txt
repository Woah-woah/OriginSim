cmake_minimum_required(VERSION 3.10)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

add_library(scantext_core STATIC
    src/ScanContext.cpp
)
target_include_directories(scantext_core PUBLIC include)
target_link_libraries(scantext_core ${PCL_LIBRARIES})

add_library(scantext_mapping_core STATIC
    src/Mapping.cpp
)
target_include_directories(scantext_mapping_core PUBLIC include)
target_link_libraries(scantext_mapping_core scantext_core ${PCL_LIBRARIES} yaml-cpp)


if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(test_scantext test/test_scantext.cpp)
  target_link_libraries(test_scantext scantext_core GTest::GTest GTest::Main ${PCL_LIBRARIES})
  add_test(NAME test_scantext COMMAND test_scantext)
endif()
