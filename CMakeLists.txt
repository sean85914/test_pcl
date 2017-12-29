cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(outlier)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (outlier statistical_removal.cpp)
target_link_libraries (outlier ${PCL_LIBRARIES})	

add_executable (cluster_extraction cluster_extraction.cpp)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})	
