cmake_minimum_required(VERSION 2.8)

project(detect)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

find_package(Eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

find_package(PCL 1.7 REQUIRED REQUIRED COMPONENTS io registration segmentation visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS   "-std=c++11")

find_package(OpenCV  REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_executable (pointcloud pointcloud.cpp mba/mba.hpp)
target_link_libraries (pointcloud ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(detect detect.cpp)
add_executable( abc abc.cpp )
target_link_libraries( abc ${CERES_LIBRARIES} ${OpenCV_LIBS} )

target_link_libraries(detect ${OpenCV_LIBS})








