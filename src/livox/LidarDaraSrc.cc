#include <iostream>

#include "LidarDataSrc.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::string;

using livox::LidarDataSrc;

LidarDataSrc& LidarDataSrc::GetInstance(){
    std::lock_guard<std::mutex> lk(mutex_);

    if(!instance_ptr_){
        instance_ptr_.reset(new LidarDataSrc);
    }

    return *instance_ptr_;
}

bool LidarDataSrc::Initialize(const std::vector<std::string> &broadcast_codes){
    if(is_initialized_){
        cerr << "LiDAR data source already initialized" << endl;
        return false;
    }

    if(!Init()){
        Uninit();
        cerr << "Livox SDK fail to init" << endl;
        return false;
    }

    LivoxSdkVersion sdk_ver;
    GetLivoxSdkVersion(&sdk_ver);
    cout << "Livox SDK version " 
         << sdk_ver.major << "." << sdk_ver.minor << "." << sdk_ver.patch << endl;

    SetBroadcastCallback(OnDeviceBroadcast);
    
}

void LidarDataSrc::OnDeviceBroadcast(const BroadcastDeviceInfo *info){
    if(info == nullptr){
        return;
    }

    if(info->dev_type == kDeviceTypeHub){
        cerr << "Expecting a standalone LiDAR instead pf a hub: " 
             << string(info->broadcast_code) << endl;
        return;
    }

    if(instance_ptr_->IsAutoConnet()){
        cout << "In auto connect mode, connecting to " 
             << string(info->broadcast_code) << endl;
    }else{
        if(!instance_ptr_->QueryWhiteList(info->broadcast_code)){
            cerr << "Not in white list: " << string(info->broadcast_code) << endl;
            return;
        }
    }

    livox_status result = kStatusFailure;
    uint8_t handle = 0;
    result = AddLidarToConnect(info->broadcast_code, &handle);
    if(result == kStatusSuccess && handle < kMaxLidarCount){
        SetDataCallback(handle, GetLidarDataCb, (void*)instance_ptr_.get());
        LidarDevice& lidar = instance_ptr_->lidars_[handle];
        lidar.handle = handle;
        lidar.con_state = kConnectStateOff;
        lidar.config.enable_fan = true;
        lidar.config.return_mode = kStrongestReturn;
        lidar.config.coordinate = kCoordinateCartesian;
        lidar.config.imu_rate = kImuFreq200Hz;
    }else{
        cerr << "Fail to add LiDAR, state: " << result << " handle: " << handle << endl;
    }
}

bool LidarDataSrc::QueryWhiteList(const char* bd_code) const{
    if(!bd_code){
        return false;
    }

    string bd_code_str(bd_code);
    for(auto code : white_list_){
        if(bd_code_str == code){
            return true;
        }
    }

    return false;
}