add_library(livox_driver livox/LidarDataSrc.cc)
target_link_libraries(livox_driver /usr/local/lib/liblivox_sdk_static.a)