#include <ros/ros.h>
#include "hikrobot_images_saver.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "hikrobot_images_save");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    try {
        HikrobotImageSaver image_saver(nh, nh_private);
    } catch (const std::exception& e) {
        ROS_FATAL("Exception in HikrobotImageSaver: %s", e.what());
        return 1;
    }

    return 0;
}