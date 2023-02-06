cmake_minimum_required(VERSION 3.10)

project(Transformation)

find_package(Eigen3 REQUIRED)

add_executable(Transformation Main.cc)
target_include_directories(Transformation PRIVATE ${EIGEN3_INCLUDE_DIR})
