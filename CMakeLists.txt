cmake_minimum_required(VERSION 3.12)
project(UDP_Port_Reuse_Example)


set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

# 生成可执行文件
add_executable(serial_port_send
    test_send.cpp
)

add_executable(serial_port_recv
    test_recv.cpp
)
