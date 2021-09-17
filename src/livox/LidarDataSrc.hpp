#pragma once

#include <mutex>
#include <memory>
#include <vector>
#include <string>
#include <array>

#include <livox_sdk.h>
#include <livox_def.h>

namespace livox{

enum LidarConnectState{
    kConnectStateOff = 0,
    kConnectStateOn = 1,
    kConnectStateConfig = 2,
    kConnectStateSampling = 3,
};

enum CoordinateType{
  kCoordinateCartesian = 0,
  kCoordinateSpherical
};

enum LidarConfigCodeBit{
    kConfigFan = 1,
    kConfigReturnMode = 2,
    kConfigCoordinate = 4,
    kConfigImuRate = 8
};

struct UserConfig{
    bool enable_fan;
    uint32_t return_mode;
    uint32_t coordinate;
    uint32_t imu_rate;
    volatile uint32_t set_bits;
    volatile uint32_t get_bits;
};

struct LidarDevice{
    uint8_t handle;
    LidarConnectState con_state;
    DeviceInfo dev_info;
    UserConfig config;
};

// Thread-safe singleton
class LidarDataSrc{
public:
    LidarDataSrc(LidarDataSrc &other) = delete;

    void operator=(const LidarDataSrc &) = delete;

    bool Initialize(const std::vector<std::string> &broadcast_codes);

    static LidarDataSrc& GetInstance();

private:

    LidarDataSrc() = default;
    ~LidarDataSrc();

    // inline bool IsAutoConnet() const{return auto_connect_;}

    bool AddBroadcastCodeToWhitelist(const std::string &bc_code);
    bool QueryWhiteList(const char* bc_code) const;
    bool QueryWhiteList(const std::string &bc_code) const;

    static void GetLidarDataCb(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data);
    static void DeviceInformationCb(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *clent_data);
    static void LidarErrorStatusCb(livox_status status, uint8_t handle, ErrorMessage *message);
    static void StartSampleCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);

    static void OnDeviceBroadcast(const BroadcastDeviceInfo * const info);
    static void OnDeviceStateChange(const DeviceInfo *info, DeviceEvent type);

    static void SetCoordinateCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);
    static void SetPointCloudReturnModeCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);
    static void SetImuRatePushFrequencyCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);

    std::vector<std::string> white_list_;
    std::array<LidarDevice, kMaxLidarCount> lidars_;
    std::array<uint32_t, kMaxLidarCount> data_recv_cnt_;

    bool auto_connect_ = false;

    volatile bool is_initialized_ = false;

    static std::unique_ptr<LidarDataSrc> instance_ptr_;
    static std::mutex mutex_;
};

}