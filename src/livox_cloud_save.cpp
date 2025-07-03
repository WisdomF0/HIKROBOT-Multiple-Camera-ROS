#include <ros/ros.h>
#include "livox_cloud_saver.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_cloud_save");
    ros::NodeHandle nh("~");

    ROS_INFO("Livox PointCloud Saver node started");
    
    try {
        // 修改LivoxPointCloudSaver构造函数，接收私有命名空间的NodeHandle
        LivoxPointCloudSaver saver(nh);
        saver.run();
    } catch (const std::exception& e) {
        ROS_FATAL("Exception caught: %s", e.what());
        return 1;
    } catch (...) {
        ROS_FATAL("Unknown exception caught");
        return 1;
    }
    
    ROS_INFO("Livox PointCloud Saver node stopped");
    return 0;
}