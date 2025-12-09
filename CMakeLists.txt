cmake_minimum_required(VERSION 3.10.0)
project(dynamic_cloud_filter)

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-Wno-deprecated-declarations -O3 -g -pthread")

## Enable CUDA
enable_language(CUDA)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf
  tf2_ros
  pcl_ros
  pcl_conversions
  jsk_recognition_msgs
  dynamic_reconfigure
)

## Find PCL
find_package(PCL REQUIRED)

## Find CUDA
if(NOT DEFINED CUDA_TOOLKIT_ROOT_DIR)
    set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
endif()

message(STATUS "Using CUDA toolkit at: ${CUDA_TOOLKIT_ROOT_DIR}")

## TensorRT path (MODIFY THIS TO YOUR TENSORRT PATH)
set(TENSORRT_ROOT /usr/local/TensorRT-8.5.3.1 CACHE STRING "TensorRT install path")
set(TENSORRT_INCLUDE_DIRS ${TENSORRT_ROOT}/include/)
set(TENSORRT_LIBRARY_DIRS ${TENSORRT_ROOT}/lib)

message(STATUS "TensorRT root: ${TENSORRT_ROOT}")
message(STATUS "TensorRT include: ${TENSORRT_INCLUDE_DIRS}")
message(STATUS "TensorRT lib: ${TENSORRT_LIBRARY_DIRS}")

## spconv library path (from TRLO)
set(arch ${CMAKE_HOST_SYSTEM_PROCESSOR})
set(SPCONV_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/libspconv/lib/${arch})

if(EXISTS ${SPCONV_LIB_DIR})
    message(STATUS "Found spconv library at: ${SPCONV_LIB_DIR}")
else()
    message(WARNING "spconv library not found at: ${SPCONV_LIB_DIR}")
    message(WARNING "Please copy 3rdparty/libspconv from TRLO project")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_centerpoint ${PROJECT_NAME}_filter
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf
    tf2_ros
    pcl_ros
    pcl_conversions
    jsk_recognition_msgs
  DEPENDS PCL
)

###########
## Build ##
###########

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CUDA_TOOLKIT_ROOT_DIR}/include
  ${TENSORRT_INCLUDE_DIRS}
  3rdparty/libspconv/include
)

## Link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${CUDA_TOOLKIT_ROOT_DIR}/lib64
  ${TENSORRT_LIBRARY_DIRS}
  ${SPCONV_LIB_DIR}
)

add_definitions(${PCL_DEFINITIONS})

## CenterPoint/PointPillar detection library (C++ and CUDA)
file(GLOB CENTERPOINT_CPP_FILES src/centerpoint/*.cpp)
file(GLOB CENTERPOINT_CU_FILES src/centerpoint/*.cu)

add_library(${PROJECT_NAME}_centerpoint
  ${CENTERPOINT_CPP_FILES}
  ${CENTERPOINT_CU_FILES}
)

target_link_libraries(${PROJECT_NAME}_centerpoint
  ${catkin_LIBRARIES}
  ${CUDA_LIBRARIES}
  cudart
  nvinfer
  nvonnxparser
  spconv
)

## Point cloud filter library
add_library(${PROJECT_NAME}_filter
  src/point_cloud_filter.cpp
  src/point_cloud_utils.cpp
  src/spatial_hash_filter.cpp
)

target_link_libraries(${PROJECT_NAME}_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## CUDA filter library
add_library(${PROJECT_NAME}_cuda_filter
  src/cuda_filter.cu
)

target_link_libraries(${PROJECT_NAME}_cuda_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}
  cudart
)

## Main node: Detection + Filter (Complete Pipeline)
add_executable(detection_filter_node
  src/detection_filter_node.cpp
)

target_link_libraries(detection_filter_node
  ${PROJECT_NAME}_centerpoint
  ${PROJECT_NAME}_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}
  cudart
  nvinfer
  nvonnxparser
  spconv
)

## Standalone filter node (subscribes to external detections)
add_executable(dynamic_filter_node
  src/dynamic_filter_node.cpp
)

target_link_libraries(dynamic_filter_node
  ${PROJECT_NAME}_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Format converter utility
add_executable(cloud_format_converter
  src/cloud_format_converter.cpp
)

target_link_libraries(cloud_format_converter
  ${PROJECT_NAME}_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Benchmark tool for performance testing
add_executable(benchmark_filter
  src/benchmark_filter.cpp
)

target_link_libraries(benchmark_filter
  ${PROJECT_NAME}_filter
  ${PROJECT_NAME}_cuda_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS
  detection_filter_node
  dynamic_filter_node
  cloud_format_converter
  ${PROJECT_NAME}_centerpoint
  ${PROJECT_NAME}_filter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Mark config files for installation
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

## Mark model directory
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
  USE_SOURCE_PERMISSIONS
)
