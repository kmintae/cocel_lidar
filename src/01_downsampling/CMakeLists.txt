cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(01_downsampling)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (voxel src/voxelization.cpp)
add_executable (uniform src/uniformsampling.cpp)
# add_executable (convert src/convert.cpp)


target_link_libraries (voxel ${PCL_LIBRARIES})
target_link_libraries (uniform ${PCL_LIBRARIES})
# target_link_libraries (convert ${PCL_LIBRARIES})
