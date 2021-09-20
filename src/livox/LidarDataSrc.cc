#include <iostream>
#include <unordered_map>
#include <cstring>

#include "livox/LidarDataSrc.hpp"
#include "livox/utils/utils.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::string;

using livox::LidarDataSrc;

std::unique_ptr<LidarDataSrc> LidarDataSrc::instance_ptr_;
std::mutex LidarDataSrc::instance_mutex_;

namespace{
    // Ref: https://github.com/Livox-SDK/livox_ros_driver/blob/880c46a91aaa602dbecf20e204da4751747b3826/livox_ros_driver/livox_ros_driver/lds.h#L238
    // Seems that once PointDataType is known, point number per package is fixed,
    // thus we can pre-compute the package size

    // Map point type to package size in byte
    const std::unordered_map<uint8_t, uint32_t> kPtypeToPkgSizeMap{
        {kCartesian, 1318},
        {kSpherical, 918},
        {kExtendCartesian, 1362},
        {kExtendSpherical, 978},
        {kDualExtendCartesian, 1362},
        {kDualExtendSpherical, 786},
        {kImu, 42},
        {kTripleExtendCartesian, 1278},
        {kTripleExtendSpherical, 678}
    };
}

LidarDataSrc::~LidarDataSrc(){
    if(is_initialized_){
        Uninit();
        cout << "Livox-SDK uninit success" << endl;
    }else{
        cerr << "Deconstructor called before Livox-SDK init" << endl;
    }
}

LidarDataSrc& LidarDataSrc::GetInstance(){
    std::lock_guard<std::mutex> lk(instance_mutex_);

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
    using std::chrono::high_resolution_clock;
    using std::chrono::duration;
    using std::chrono::duration_cast;

    static bool first_enter = true;
    static high_resolution_clock::time_point last_frame_end;

    // clinet_data is not used
    if(!data || !data_num || (handle >= kMaxLidarCount)){
        return;
    }

    if(first_enter){
        last_frame_end = high_resolution_clock::now();
        first_enter = false;
    }

    // Unit: second
    duration<double> time_elapse = duration_cast<duration<double>>(high_resolution_clock::now() - last_frame_end);
    if(time_elapse.count() > instance_ptr_->intgrate_time_){
        {
            std::lock_guard<std::mutex> lk(instance_ptr_->dbuf_mutex_);
            std::swap(instance_ptr_->prepare_data_q_, instance_ptr_->ready_data_q_);
        }

        // Empty the queue for data preparing
        if(!instance_ptr_->prepare_data_q_.empty()){
            std::queue<RawDataPkg> empty_q;
            std::swap(instance_ptr_->prepare_data_q_, empty_q);
        }

        last_frame_end = high_resolution_clock::now();

        // cout << "Double buffer swapped, new time: " << last_frame_end.time_since_epoch().count() << endl;
    }
    
    RawDataPkg pkg;
    pkg.pt_num = data_num;
    std::memcpy(pkg.raw_data, (uint8_t *)data, kPtypeToPkgSizeMap.find(data->data_type)->second);

    // cout << "Data number: " << data_num << " expected size: " << kPtypeToPkgSizeMap.find((PointDataType)data->data_type)->second << endl;

    instance_ptr_->prepare_data_q_.push(pkg);
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
        cout << "Firmware version: " << (unsigned)ack->firmware_version[0] << "."
                                     << (unsigned)ack->firmware_version[1] << "."
                                     << (unsigned)ack->firmware_version[2] << "."
                                     << (unsigned)ack->firmware_version[3] << endl; 
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

std::vector<livox::PointXYZR> LidarDataSrc::GetPtdataXYZR(){ 
    std::queue<RawDataPkg> raw_queue;
    {// Copy data to local variable
        std::lock_guard<std::mutex> lk(dbuf_mutex_);
        raw_queue = ready_data_q_;
    }

    std::vector<livox::PointXYZR> out;
    while(!raw_queue.empty()){
        RawDataPkg p = raw_queue.front();
        LivoxEthPacket *raw_pkg = reinterpret_cast<LivoxEthPacket *>(p.raw_data);
        uint64_t timestamp = *reinterpret_cast<uint64_t *>(raw_pkg->timestamp);
        LivoxExtendRawPointToPointXYZR(out, raw_pkg->data, p.pt_num, timestamp);

        raw_queue.pop();
    }

    return out;
}

void LidarDataSrc::LivoxExtendRawPointToPointXYZR(std::vector<PointXYZR> &out, uint8_t *pt_data, uint32_t pt_num, uint64_t timestamp) const{
    using livox::utils::IsVec3Zero;

    LivoxExtendRawPoint *raw_pts = reinterpret_cast<LivoxExtendRawPoint *>(pt_data);
    for(size_t i = 0; i < pt_num; ++i){
        // Check if zero
        if(IsVec3Zero(raw_pts[i].x, raw_pts[i].y, raw_pts[i].z)){
            continue;
        }

        // Convert from mm to m
        out.emplace_back(PointXYZR{raw_pts[i].x / 1000.0f, 
                                   raw_pts[i].y / 1000.0f, 
                                   raw_pts[i].z / 1000.0f, 
                                   (float)raw_pts[i].reflectivity});
    }

}