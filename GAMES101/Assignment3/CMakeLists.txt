cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_executable(Rasterizer
  Main.cc
  Mesh.cc
  Mesh.h
  Rasterizer.cc
  Rasterizer.h
  Texture.cc
  Texture.h
)
target_include_directories(Rasterizer PRIVATE ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(Rasterizer PRIVATE ${OpenCV_LIBRARIES})
