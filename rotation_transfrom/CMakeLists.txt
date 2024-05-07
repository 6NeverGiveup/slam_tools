cmake_minimum_required(VERSION 2.8)
project(rotation_transfrom)

# 查找PCL库并添加头文件和链接库
find_package(Eigen3 REQUIRED)
include_directories(include ${EIGEN3_INCLUDE_DIR})

# 添加可执行文件
add_executable(rotation_transfrom rotation_transfrom.cpp)

# 链接PCL库
target_link_libraries(rotation_transfrom ${EIGEN3_LIBRARIES})