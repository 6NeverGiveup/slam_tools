cmake_minimum_required(VERSION 2.8)
project(play_pcd)

# 查找PCL库并添加头文件和链接库
find_package(PCL REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件
add_executable(play_pcd play_pcd.cpp)

# 链接PCL库
target_link_libraries(play_pcd ${PCL_LIBRARIES})