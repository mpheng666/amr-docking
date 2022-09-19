cmake_minimum_required(VERSION 3.0.2)
project(target_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  PCL REQUIRED
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES target_localization
 CATKIN_DEPENDS 
  roscpp
 DEPENDS
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_clustering_node 
  src/pc_clustering.cpp
)
target_link_libraries (pcl_clustering_node 
  ${PCL_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${catkin_LIBRARIES}
)
