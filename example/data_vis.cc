#include <vector>
#include <chrono>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>

#include "livox/LidarDataSrc.hpp"

pcl::PointCloud<pcl::PointXYZI>::Ptr ToPclPtcloud(const std::vector<livox::PointXYZR> &ptcloud);

int main(int argc, char const *argv[]){
    livox::LidarDataSrc& data_src = livox::LidarDataSrc::GetInstance();
    data_src.Initialize();

    pcl::visualization::CloudViewer viewer("Ptcloud");

    while(!viewer.wasStopped()){
        std::vector<livox::PointXYZR> out = data_src.GetPtdataXYZR();
        pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud = ToPclPtcloud(out);

        viewer.showCloud(vis_cloud);
    }

    return EXIT_SUCCESS;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ToPclPtcloud(const std::vector<livox::PointXYZR> &ptcloud){
    pcl::PointCloud<pcl::PointXYZI>::Ptr res(new pcl::PointCloud<pcl::PointXYZI>);
    for(size_t i = 0; i < ptcloud.size(); ++i){
        pcl::PointXYZI p;
        p.x = ptcloud[i].x;
        p.y = ptcloud[i].y;
        p.z = ptcloud[i].z;
        p.intensity = ptcloud[i].reflectivity;
        res->points.push_back(p);
        // std::cout << p.x << " " << p.y << " " << p.z << std::endl;
    }
    res->width = res->size();
    res->height = 1;
    res->is_dense = false;

    // std::cout << "Pt number: " << res->size() << std::endl;

    return res;
}