cmake_minimum_required(VERSION 3.0)
project(colorful_print)

# 添加可执行文件
add_executable(colorful_print colorful_print.cpp)

# 指定链接的标准 C++ 库
target_link_libraries(colorful_print stdc++)

# 如果你的系统支持 ANSI 转义码，不需要其他特殊配置
