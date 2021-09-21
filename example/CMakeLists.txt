add_executable(get_data get_data.cc)
target_link_libraries(get_data livox_driver Threads::Threads)

if(BUILD_VISUAL_EXAMPLE)
    add_executable(data_vis data_vis.cc)
    target_link_libraries(data_vis livox_driver Threads::Threads ${PCL_LIBRARIES})
endif()