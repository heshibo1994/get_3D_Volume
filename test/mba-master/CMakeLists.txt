cmake_minimum_required(VERSION 3.9)
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type")
endif()
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(mba)

find_package(Eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

find_package(PCL 1.7 REQUIRED REQUIRED COMPONENTS io registration segmentation visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_executable(2d 2d.cpp mba/mba.hpp)
add_executable(2d_execute 2d_execute.cpp mba/mba.hpp)
target_link_libraries (2d_execute ${PCL_LIBRARIES})
enable_testing()
add_subdirectory(python)
add_subdirectory(tests)





