cmake_minimum_required(VERSION 3.8)
project(fast_lio_localization2)

# Set default build type to Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

ADD_COMPILE_OPTIONS(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17 -O3")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++0x -std=c++17 -fexceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")

  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
# find_package(livox_ros_driver2 REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(dependencies
  rclcpp
  rclpy
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_ros
  pcl_conversions
#  livox_ros_driver2
)

# Thirdparty libraries
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

message(Eigen: ${EIGEN3_INCLUDE_DIR})
message(STATUS "PCL: ${PCL_INCLUDE_DIRS}")

set(msg_files
  "msg/Pose6D.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_export_dependencies(rosidl_default_runtime)

# add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
add_library(fast_lio_mapping_comp src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
target_include_directories(fast_lio_mapping_comp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(fast_lio_mapping_comp 
  ${PCL_LIBRARIES} 
  ${PYTHON_LIBRARIES} 
  Eigen3::Eigen
  "${cpp_typesupport_target}"
)
target_include_directories(fast_lio_mapping_comp PRIVATE ${PYTHON_INCLUDE_DIRS})
target_compile_features(fast_lio_mapping_comp PUBLIC cxx_std_17)
ament_target_dependencies(fast_lio_mapping_comp 
  rclcpp 
  rclcpp_components
  # rclcpp_lifecycle
  lifecycle_msgs
  std_srvs 
  sensor_msgs 
  nav_msgs 
  geometry_msgs 
  visualization_msgs
  tf2 
  tf2_geometry_msgs 
  tf2_eigen 
  pcl_conversions
  nav2_util
)

rclcpp_components_register_node(fast_lio_mapping_comp
  PLUGIN "fast_lio_mapping::LaserMapping"
  EXECUTABLE fast_lio_mapping_node
)

add_library(fast_lio_localization_comp SHARED src/fast_lio_localization.cpp)
target_include_directories(fast_lio_localization_comp PRIVATE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(fast_lio_localization_comp PUBLIC cxx_std_17)
ament_target_dependencies(fast_lio_localization_comp 
  rclcpp 
  rclcpp_components 
  # rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_geometry_msgs 
  tf2_eigen 
  pcl_conversions
  nav2_util
)
target_link_libraries(fast_lio_localization_comp ${PCL_LIBRARIES})

rclcpp_components_register_node(fast_lio_localization_comp
  PLUGIN "fast_lio_localization::FastLIOLocalization"
  EXECUTABLE fast_lio_localization_node
)

add_library(transform_fusion_comp SHARED src/transform_fusion.cpp)
target_include_directories(transform_fusion_comp PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(transform_fusion_comp PUBLIC cxx_std_17)
ament_target_dependencies(transform_fusion_comp 
  rclcpp 
  rclcpp_components 
  # rclcpp_lifecycle
  lifecycle_msgs
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_geometry_msgs 
  tf2_ros
  nav2_util
)

# Register the Transform Fusion component
rclcpp_components_register_node(transform_fusion_comp
  PLUGIN "fast_lio_localization::TransformFusion"
  EXECUTABLE transform_fusion_node
)

install(
  DIRECTORY 
  config 
  launch 
  map
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  fast_lio_mapping_comp
  fast_lio_localization_comp
  transform_fusion_comp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  # fast_lio_localization2/global_localization.py
  fast_lio_localization2/publish_initial_pose.py
  # fast_lio_localization2/transform_fusion.py
  # fast_lio_localization2/invert_livox_scan.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()