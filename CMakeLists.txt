cmake_minimum_required(VERSION 3.22)

project(vicon2gt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time serialization regex timer)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)

# GTSAM 4.2a5 uses GTSAM::GTSAM target
if(TARGET GTSAM::GTSAM)
  set(GTSAM_TARGET GTSAM::GTSAM)
elseif(TARGET gtsam)
  set(GTSAM_TARGET gtsam)
else()
  message(FATAL_ERROR "GTSAM target not found!")
endif()

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Include our header files
include_directories(
  src
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# Build library
add_library(vicon2gt_lib SHARED
  src/gtsam/JPLNavState.cpp
  src/gtsam/JPLQuaternion.cpp
  src/gtsam/RotationXY.cpp
  src/gtsam/ImuFactorCPIv1.cpp
  src/gtsam/MeasBased_ViconPoseTimeoffsetFactor.cpp
  src/meas/Interpolator.cpp
  src/meas/Propagator.cpp
  src/sim/BsplineSE3.cpp
  src/sim/Simulator.cpp
  src/solver/ViconGraphSolver.cpp
)

target_link_libraries(vicon2gt_lib
  Eigen3::Eigen
  ${Boost_LIBRARIES}
  ${GTSAM_TARGET}
  yaml-cpp
)

target_include_directories(vicon2gt_lib PUBLIC src)

# Build executables
add_executable(estimate_vicon2gt src/estimate_vicon2gt.cpp)
target_link_libraries(estimate_vicon2gt vicon2gt_lib)
ament_target_dependencies(estimate_vicon2gt
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  geometry_msgs
  sensor_msgs
  nav_msgs
  rosidl_typesupport_cpp
)

add_executable(run_simulation src/run_simulation.cpp)
target_link_libraries(run_simulation vicon2gt_lib)
ament_target_dependencies(run_simulation
  rclcpp
  geometry_msgs
  nav_msgs
)

# Install targets
install(TARGETS vicon2gt_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS estimate_vicon2gt run_simulation
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python scripts
install(PROGRAMS scripts/trajectory_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rclcpp rosbag2_cpp rosbag2_storage geometry_msgs sensor_msgs nav_msgs Eigen3 Boost GTSAM)

ament_package()