cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(01_filter)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (stats src/stats.cpp)
add_executable (radius src/radius.cpp)
# add_executable (convert src/convert.cpp)


target_link_libraries (stats ${PCL_LIBRARIES})
target_link_libraries (radius ${PCL_LIBRARIES})
# target_link_libraries (convert ${PCL_LIBRARIES})
