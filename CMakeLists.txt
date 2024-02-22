cmake_minimum_required(VERSION 2.6)

project(trt_det LANGUAGES CXX)

add_definitions(-std=c++11)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


find_package(Eigen3 REQUIRED)
include_directories(/usr/include/eigen3)


add_executable(ucmc_track
    src/main.cpp
    include/image_utils/lap/lapjv.cpp
    include/image_utils/lap/lapmod.cpp
)

target_link_libraries(ucmc_track yaml-cpp)
target_link_libraries(ucmc_track ${OpenCV_LIBS})
