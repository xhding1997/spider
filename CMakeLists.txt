cmake_minimum_required(VERSION 3.17)
project(bodyctrl)

set(CMAKE_CXX_STANDARD 14)

add_executable(bodyctrl main.cpp matrix.cpp coordinates_ctrl.cpp reachable.cpp communication.cpp angle_ctrl.cpp angle_ctrl.h matrix.h coordinates_ctrl.h reachable.h)