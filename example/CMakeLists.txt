add_executable(get_data get_data.cc)
target_link_libraries(get_data livox_driver Threads::Threads)