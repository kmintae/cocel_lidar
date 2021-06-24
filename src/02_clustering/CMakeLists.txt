cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(02_clustering)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (euclidean src/euclidean.cpp)
# add_executable (conditional src/conditional.cpp)
# add_executable (convert src/convert.cpp)


target_link_libraries (euclidean ${PCL_LIBRARIES})
# target_link_libraries (conditional ${PCL_LIBRARIES})
# target_link_libraries (convert ${PCL_LIBRARIES})
