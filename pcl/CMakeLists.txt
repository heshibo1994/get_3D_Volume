cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(add_plane)

find_package(PCL 1.7 REQUIRED REQUIRED COMPONENTS io registration segmentation visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable (add_plane add_plane.cpp)
target_link_libraries (add_plane ${PCL_LIBRARIES})



add_executable (read_ply read_ply.cpp)
target_link_libraries (read_ply ${PCL_LIBRARIES})


