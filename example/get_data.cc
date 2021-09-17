#include <vector>
#include <string>
#include <chrono>
#include <thread>

#include "livox/LidarDataSrc.hpp"

int main(int argc, char const *argv[]){

    std::vector<std::string> white_list;

    livox::LidarDataSrc& data_src = livox::LidarDataSrc::GetInstance();
    data_src.Initialize(white_list);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    return EXIT_SUCCESS;
}
