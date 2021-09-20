#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <iostream>

#include "livox/LidarDataSrc.hpp"

int main(int argc, char const *argv[]){
    livox::LidarDataSrc& data_src = livox::LidarDataSrc::GetInstance();
    data_src.Initialize();

    while(true){
        std::vector<livox::PointXYZR> out = data_src.GetPtdataXYZR();
        std::cout << "Point number: " << out.size() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return EXIT_SUCCESS;
}
