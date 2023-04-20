# Set the minimum version of CMake that can be used
cmake_minimum_required(VERSION 3.0)

# project name
project(EDGstar_Pathfinding)

# head file path
# Add include directories to the build.
# 头文件路径
#include_directories( include )
# include_directories( ./include )
include_directories( ${CMAKE_SOURCE_DIRE}/include )

# Add a subdirectory to the build.
# add_subdirectory( src DIR_SRCS )

find_package(OpenCV REQUIRED)
include_directories(
  include ${catkin_INCLUDE_DIRS}${OpenCV_INCLUDE_DIRS})
target_link_libraries(excutable_file_name
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(EDGstar_Pathfinding main.cpp src/EDG_Star.cpp src/ScenarioLoader.cpp)

