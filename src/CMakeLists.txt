add_library(
    livox_driver STATIC 
        livox/LidarDataSrc.cc
        livox/utils/utils.cc
)
target_link_libraries(
    livox_driver 
        ${LivoxSDK_LIBRARY}
)