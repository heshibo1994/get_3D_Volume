cmake_minimum_required(VERSION 2.8)

project(detect)

find_package(Eigen3)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS   "-std=c++11")

find_package(OpenCV  REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(detect detect.cpp)

target_link_libraries(detect ${OpenCV_LIBS})
