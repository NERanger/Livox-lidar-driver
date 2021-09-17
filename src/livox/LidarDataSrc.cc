#include <iostream>

#include "livox/LidarDataSrc.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::string;

using livox::LidarDataSrc;

LidarDataSrc::~LidarDataSrc(){
    if(is_initialized_){
        Uninit();
        cout << "Livox-SDK uninit success" << endl;
    }

    cerr << "Deconstructor called before Livox-SDK init" << endl;
}

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
    SetDeviceStateUpdateCallback(OnDeviceStateChange);

    // Add white list
    for(auto bc_code : broadcast_codes){
        AddBroadcastCodeToWhitelist(bc_code);
    }

    if(!white_list_.empty()){
        auto_connect_ = false;
        cout << "Auto connect disabled, white list:" << endl;
        for(auto bc_code : white_list_){
            cout << bc_code << endl;
        }
    }else{
        auto_connect_ = true;
        cout << "White list empty, enable auto connect" << endl;
    }

    // Start livox sdk to receive lidar data
    if(!Start()){
        Uninit();
        cout << "Fail to init Livox-SDK" << endl;
        return false;
    }

    is_initialized_ = true;
    cout << "Livox-SDK init success" << endl;

    return true;
}

bool LidarDataSrc::AddBroadcastCodeToWhitelist(const std::string &bc_code){
    if(bc_code.empty() || bc_code.length() > kBroadcastCodeSize){
        cerr << "Invalid broadcast code" << endl;
        return false;
    }

    if(white_list_.size() > kMaxLidarCount){
        cerr << "White list exceed max size: " << kMaxLidarCount << endl;
        return false;
    }

    if(QueryWhiteList(bc_code)){
        cerr << "Broadcast code already exist" << endl;
        return false;
    }

    white_list_.emplace_back(bc_code);

    return true;
}

void LidarDataSrc::GetLidarDataCb(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data){
    // clinet_data is not used
    if(!data || !data_num || (handle >= kMaxLidarCount)){
        return;
    }

    if(data){
        instance_ptr_->data_recv_cnt_[handle] += 1;  // handle is expected in [0,kMaxLidarCount)
        if(instance_ptr_->data_recv_cnt_[handle] % 100 == 0){
            cout << "Recv pkg count: " << handle << " " << instance_ptr_->data_recv_cnt_[handle] << endl;
        }
    }
}

void LidarDataSrc::OnDeviceBroadcast(const BroadcastDeviceInfo * const info){
    if(info == nullptr){
        return;
    }

    if(info->dev_type == kDeviceTypeHub){
        cerr << "Expecting a standalone LiDAR instead pf a hub: " 
             << string(info->broadcast_code) << endl;
        return;
    }

    if(instance_ptr_->auto_connect_){
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

void LidarDataSrc::OnDeviceStateChange(const DeviceInfo *info, DeviceEvent type){
    if(info == nullptr){
        return;
    }

    uint8_t handle = info->handle;
    if(handle >= kMaxLidarCount){
        return;
    }

    LidarDevice& lidar = instance_ptr_->lidars_[handle];
    if(type == kEventConnect){
        QueryDeviceInformation(handle, DeviceInformationCb, (void *)instance_ptr_.get());
        if(lidar.con_state == kConnectStateOff){
            lidar.con_state = kConnectStateOn;
            lidar.dev_info = *info;
        }
        cout << "[WARNING] LiDAR sn [" << string(info->broadcast_code) 
                                       << "] connected" << endl;
    }else if(type == kEventDisconnect){
        lidar.con_state = kConnectStateOff;
        cout << "[WARNING] LiDAR sn [" << string(info->broadcast_code)
                                       << "] disconnected" << endl;
    }else if(type == kEventStateChange){
        lidar.dev_info = *info;
        cout << "[WARNING] LiDAR sn [" << string(info->broadcast_code)
                                       << "] state change" << endl;
    }

    if(lidar.con_state == kConnectStateOn){
        cout << "Device working state: " << lidar.dev_info.state << endl;
        if(lidar.dev_info.state == kLidarStateInit){
            cout << "Device state change progress: " 
                 << lidar.dev_info.status.progress << endl;
        }else{
            cout << "Device state error code: " 
                 << lidar.dev_info.status.status_code.error_code << endl;
        }
        cout << "Device feature " << lidar.dev_info.feature << endl;

        SetErrorMessageCallback(handle, LidarErrorStatusCb);

        // Config LiDAR param
        if(lidar.dev_info.state == kLidarStateNormal){
            if(lidar.config.coordinate != 0){
                SetSphericalCoordinate(handle, SetCoordinateCb, instance_ptr_.get());
            }else{
                SetCartesianCoordinate(handle, SetCoordinateCb, instance_ptr_.get());
            }

            lidar.config.set_bits |= kConfigCoordinate;

            if(info->type != kDeviceTypeLidarMid40){
                LidarSetPointCloudReturnMode(handle, (PointCloudReturnMode)(lidar.config.return_mode),
                                             SetPointCloudReturnModeCb, (void *)instance_ptr_.get());
                lidar.config.set_bits |= kConfigReturnMode;
            }

            if(info->type != kDeviceTypeLidarMid40 && info->type != kDeviceTypeLidarMid70){
                LidarSetImuPushFrequency(handle, (ImuFreq)(lidar.config.imu_rate), 
                                         SetImuRatePushFrequencyCb, (void *)instance_ptr_.get());
                lidar.config.set_bits |= kConfigImuRate;
            }

            lidar.con_state = kConnectStateConfig;
        }
    }
}

void LidarDataSrc::SetImuRatePushFrequencyCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data){
    // client_data not used
    if(handle >= kMaxLidarCount){
        return;
    }

    LidarDevice& lidar = instance_ptr_->lidars_[handle];
    if(status == kStatusSuccess){
        lidar.config.set_bits &= ~((uint32_t)(kConfigImuRate));
        cout << "Set IMU rate success" << endl;

        if(!lidar.config.set_bits){
            LidarStartSampling(handle, StartSampleCb, (void *)instance_ptr_.get());
            lidar.con_state = kConnectStateSampling;
        }
    }else{
        LidarSetImuPushFrequency(handle, (ImuFreq)(lidar.config.imu_rate),
                                 SetImuRatePushFrequencyCb, (void *)instance_ptr_.get());
        cerr << "Fail to set IMU rate, retrying..." << endl;
    }
}

void LidarDataSrc::SetPointCloudReturnModeCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data){
    // client_data not used
    if(handle >= kMaxLidarCount){
        return;
    }

    LidarDevice& lidar = instance_ptr_->lidars_[handle];

    if(status == kStatusSuccess){
        lidar.config.set_bits &= ~((uint32_t)(kConfigReturnMode));
        cout << "Set return mode success" << endl;

        if(!lidar.config.set_bits){
            LidarStartSampling(handle, StartSampleCb, (void *)instance_ptr_.get());
            lidar.con_state = kConnectStateSampling;
        }
    }else{
        LidarSetPointCloudReturnMode(handle, (PointCloudReturnMode)(lidar.config.return_mode),
                                     SetPointCloudReturnModeCb, (void *)instance_ptr_.get());
        cerr << "Fail to set return mode, retrying..." << endl;
    }
}

void LidarDataSrc::SetCoordinateCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data){
    // client_date not used
    if(handle >= kMaxLidarCount){
        return;
    }

    LidarDevice& lidar = instance_ptr_->lidars_[handle];
    if(status == kStatusSuccess){
        lidar.config.set_bits &= ~((uint32_t)(kConfigCoordinate));
        cout << "Set coordinate success" << endl;
        if(!lidar.config.set_bits){
            LidarStartSampling(handle, StartSampleCb, instance_ptr_.get());
            lidar.con_state = kConnectStateSampling;
        }
    }else{
        if(lidar.config.coordinate != 0){
            SetSphericalCoordinate(handle, SetCoordinateCb, instance_ptr_.get());
        }else{
            SetCartesianCoordinate(handle, SetCoordinateCb, instance_ptr_.get());
        }

        cout << "Fail to set coordinate, retrying..." << endl;
    }
}

void LidarDataSrc::StartSampleCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data){
    // client_data not used
    if(handle >= kMaxLidarCount){
        return;
    }

    LidarDevice& lidar = instance_ptr_->lidars_[handle];
    if(status == kStatusSuccess){
        if(response != 0){
            lidar.con_state = kConnectStateOn;
            cerr << "LiDAR fail to start sampling:" 
                 << " state - " << status
                 << " handle - " << handle
                 << " response - " << response << endl;
        }else{
            cout << "LiDAR start sampling" << endl;
        }
    }else if(status == kStatusTimeout){
        lidar.con_state = kConnectStateOn;
        cerr << "Lidar start sample timeout:"
             << " state - " << status
             << " handle - " << handle
             << " response - " << response << endl;
    }
}

void LidarDataSrc::LidarErrorStatusCb(livox_status status, uint8_t handle, ErrorMessage *message){
    static uint32_t err_msg_cnt_ = 0;

    if(message != NULL){
        err_msg_cnt_ += 1;
        if(err_msg_cnt_ % 100 == 0){
            cout << "Handle: " << handle << endl;
            cout << "temp_status: " << message->lidar_error_code.temp_status << endl;
            cout << "volt_status: " << message->lidar_error_code.volt_status << endl;
            cout << "motor_status: " << message->lidar_error_code.motor_status << endl;
            cout << "dirty_warn: " << message->lidar_error_code.dirty_warn << endl;
            cout << "firmware_err: " << message->lidar_error_code.firmware_err << endl;
            cout << "pps_status: " << message->lidar_error_code.pps_status << endl;
            cout << "fan_status: " << message->lidar_error_code.fan_status << endl;
            cout << "self_heating: " << message->lidar_error_code.self_heating << endl;
            cout << "ptp_status: " << message->lidar_error_code.ptp_status << endl;
            cout << "time_sync_status: " << message->lidar_error_code.time_sync_status << endl;
            cout << "system_status: " << message->lidar_error_code.system_status << endl;
        }
    }
}

void LidarDataSrc::DeviceInformationCb(livox_status status, uint8_t handle, 
                                       DeviceInformationResponse *ack, void *clent_data){
    if(status != kStatusSuccess){
        cerr << "Fail to query device info: " << status << endl;
    }

    if(ack){
        cout << "Firmware version: " << ack->firmware_version[0] << "."
                                     << ack->firmware_version[1] << "."
                                     << ack->firmware_version[2] << "."
                                     << ack->firmware_version[3] << endl; 
    }
}

bool LidarDataSrc::QueryWhiteList(const char* bc_code) const{
    if(!bc_code){
        return false;
    }

    string bc_code_str(bc_code);
    for(auto code : white_list_){
        if(bc_code_str == code){
            return true;
        }
    }

    return false;
}

bool LidarDataSrc::QueryWhiteList(const std::string &bc_code) const{
    if(bc_code.empty()){
        return false;
    }

    for(auto code : white_list_){
        if(bc_code == code){
            return true;
        }
    }

    return false;
}