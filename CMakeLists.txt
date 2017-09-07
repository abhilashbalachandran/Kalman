cmake_minimum_required(VERSION 2.8.3)
project(map_building)

set(EIGEN_INC_DIR /usr/include/eigen3/)
include_directories(${EIGEN_INC_DIR})

add_compile_options(-std=c++11)

set(SOURCE_FILES kalman.cpp kalman_variables.cpp)
add_executable(kalman ${SOURCE_FILES})
