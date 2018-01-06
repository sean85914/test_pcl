cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(outlier)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (voxel voxel_grid.cpp)
target_link_libraries (voxel ${PCL_LIBRARIES})	

add_executable (cluster_extraction cluster_extraction.cpp)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})	

add_executable (pcl_visualizer pcd_visualization.cpp)
target_link_libraries (pcl_visualizer ${PCL_LIBRARIES})

add_executable (pcl_visual_xyz pcd_visual_xyz.cpp)
target_link_libraries (pcl_visual_xyz ${PCL_LIBRARIES})


