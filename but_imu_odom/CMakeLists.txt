cmake_minimum_required(VERSION 2.8.3)
project(but_imu_odom)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  nodelet
  pcl_ros
  sensor_msgs
  nav_msgs
  roslint
  eigen_conversions
  pcl_conversions
  tf_conversions
  tf2_eigen
  tf
)

add_definitions(-DVELODYNE_MODEL=${VELODYNE_MODEL})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)

find_package( Eigen REQUIRED )
include_directories( ${EIGEN_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories( ${PCL_LIBRARY_DIRS})

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(but_velodyne REQUIRED)
include_directories(${ButVELODYNE_INCLUDE_DIRS})
link_directories(${ButVELODYNE_LIBRARY_DIRS})

catkin_package(CATKIN_DEPENDS nav_msgs sensor_msgs tf2_eigen)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(velodyne_imu_bag_parser src/velodyne_imu_bag_parser.cpp)
target_link_libraries(velodyne_imu_bag_parser ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${ButVELODYNE_LIBRARIES})

find_package(Qt5Positioning)

add_executable(imu_gps_odom_node src/imu_gps_odom_node.cpp)
target_link_libraries(imu_gps_odom_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${ButVELODYNE_LIBRARIES} Qt5::Positioning)
