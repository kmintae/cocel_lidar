cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(01_roi)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (passthrough src/passthrough.cpp)
add_executable (conditional src/conditional.cpp)
# add_executable (convert src/convert.cpp)


target_link_libraries (passthrough ${PCL_LIBRARIES})
target_link_libraries (conditional ${PCL_LIBRARIES})
# target_link_libraries (convert ${PCL_LIBRARIES})
