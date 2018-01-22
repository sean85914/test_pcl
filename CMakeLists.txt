cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(outlier)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (voxel voxel_grid.cpp)
target_link_libraries (voxel ${PCL_LIBRARIES})	

add_executable (color color.cpp)
target_link_libraries (color ${PCL_LIBRARIES})

add_executable (cluster_extraction cluster_extraction.cpp)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})	

add_executable (removal statistical_removal.cpp)
target_link_libraries(removal ${PCL_LIBRARIES})

add_executable (pca_test pca.cpp)
target_link_libraries (pca_test ${PCL_LIBRARIES})

add_executable (pca_visual visual_pca.cpp)
target_link_libraries (pca_visual ${PCL_LIBRARIES})


