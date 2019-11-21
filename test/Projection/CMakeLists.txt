cmake_minimum_required(VERSION 2.8)

project(test)

set(CMAKE_CXX_FLAGS   "-std=c++11")

find_package(OpenCV  REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

add_executable(test test.cpp)

target_link_libraries(test ${OpenCV_LIBS} ${EIGEN3_LIBS})

