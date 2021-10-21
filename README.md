# Livox LiDAR Driver

A simple cpp driver for Livox LiDAR. No ROS requirement.

Currently Livox only provides a official grab-to-go driver with ROS. And the [cpp sample](https://github.com/Livox-SDK/Livox-SDK/tree/master/sample_cc) provided in Livox-SDK still needs extra development to get the point data. 

This repo provides a simple cpp sample based on Livox-SDK that can let user get point data conveniently.

![](https://github.com/NERanger/Livox-lidar-driver/blob/main/docs/data_visualization.png)

*Point cloud from Livox AVIA with a integration time of 0.5 seconds*

## Prerequisites

* [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK)
* Point Cloud Library (only required if you want to build visualization example)
    > sudo apt install libpcl-dev

## Build
```shell
mkdir build
# BUILD_VISUAL_EXAMPLE is default to OFF
cmake -D BUILD_VISUAL_EXAMPLE=ON ..
make
```

## Usage

Get instance for `LidarDataSrc`

```c++
livox::LidarDataSrc& data_src = livox::LidarDataSrc::GetInstance();
```

Intialize with auto-connection mode

```c++
data_src.Initialize();  // This enables auto-connect mode by default
```

Intialize with specified white-list

```c++
std::vector<std::string> white_list;
// Populate data ...
data_src.Initialize(white_list);
```

Set & get ingegrate time

```c++
data_src.SetIntegrateTime(0.1);
double t = data_src.IntegrateTime();
```

Get point data

```c++
std::vector<livox::PointXYZR> out = data_src.GetPtdataXYZR();
```

A full sample

```c++
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
```

You can find samples [here](https://github.com/NERanger/Livox-lidar-driver/tree/main/example)

## Key Function Explanation

### Integration Time

Livox LiDAR will continuously scan and return data. Integration Time is the desired time span of data integration for onr frame. The longer the time, the more data you will get when calling `GetPtdataXYZR()`.

### Double Buffer

There are 2 buffers called `prepare_data_q_` and `ready_data_q_` respectively. Every time the sdk receives data, the data will be stored in `prepare_data_q_`. Every time the integration time is reached, the frame is considered as complete and the content of `prepare_data_q_` and `ready_data_q_` will be swapped. And when you call `GetPtdataXYZR()`, the data is parsed from `ready_data_q_`.

## TODO

* Estimate timestamp for every point
* Bug fixing: current imu data is blended in lidar point data
