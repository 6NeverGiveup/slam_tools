cmake_minimum_required(VERSION 3.0.2)
project(bag2pcd)
set(CMAKE_CXX_STANDARD 17)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  rosbag
)
catkin_package(

)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} bag2pcd.cpp)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  -lstdc++fs
)
