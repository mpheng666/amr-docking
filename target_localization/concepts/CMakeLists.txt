cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(cluster_extraction)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cluster_extraction cluster_extraction.cpp)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})

add_executable (random_sample_consensus random_sample_consensus.cpp)
target_link_libraries (random_sample_consensus ${PCL_LIBRARIES})