cmake_minimum_required(VERSION 2.8.3)
project(fixed_map)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pcl_conversions
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable (hallway_map src/hallway_map.cpp )
add_executable (pcd_map src/map_from_pcd.cpp )
target_link_libraries(hallway_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})
target_link_libraries(pcd_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})
