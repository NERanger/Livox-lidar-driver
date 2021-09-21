cmake_minimum_required(VERSION 3.1)
project(livox_lidar_driver)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++17 -O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

option(BUILD_VISUAL_EXAMPLE "Build visualization example (require PCL 1.3)" OFF)

find_package(Threads REQUIRED)
find_package(LivoxSDK REQUIRED) # LivoxSDK_INCLUDE_DIR and LivoxSDK_LIBRARY

if(BUILD_VISUAL_EXAMPLE)
    find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
endif()

include_directories(
    ${LivoxSDK_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/src/
)

add_subdirectory(src)
add_subdirectory(example)