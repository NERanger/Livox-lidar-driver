cmake_minimum_required(VERSION 3.1)
project(livox_lidar_driver)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++17 -O3")

find_package(Threads REQUIRED)

include_directories(
    src/
)

add_subdirectory(src)
add_subdirectory(example)