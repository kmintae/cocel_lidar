cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(01_io)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (read src/read.cpp)
add_executable (genNwrite src/genNwrite.cpp)
# add_executable (convert src/convert.cpp)


target_link_libraries (read ${PCL_LIBRARIES})
target_link_libraries (genNwrite ${PCL_LIBRARIES})
# target_link_libraries (convert ${PCL_LIBRARIES})
