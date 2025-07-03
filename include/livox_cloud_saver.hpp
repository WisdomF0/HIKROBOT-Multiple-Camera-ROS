#ifndef LIVOX_CLOUD_SAVER_HPP
#define LIVOX_CLOUD_SAVER_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <std_msgs/Bool.h>
#include <string>
#include <boost/filesystem.hpp>
#include <mutex>

class LivoxPointCloudSaver {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;  // 添加私有命名空间的NodeHandle
    ros::Subscriber lidar_sub_;
    ros::Subscriber save_sub_;
    
    pcl::PointCloud<pcl::PointXYZI> cloud_;
    bool save_flag_;
    std::string save_path_;
    int msg_num_;
    int msg_count_;
    bool collecting_;
    std::mutex cloud_mutex_;  // 添加互斥锁
    
    std::string lidar_topic_;  // 提前声明成员变量
    std::string save_topic_;   // 提前声明成员变量

    
public:
    LivoxPointCloudSaver(ros::NodeHandle& nh) : nh_(nh), nh_private_("~") {
        // 初始化参数
        nh_private_.param<std::string>("lidar_topic", lidar_topic_, "/livox/lidar");
        nh_private_.param<std::string>("save_topic", save_topic_, "/save_image");
        nh_private_.param<int>("msg_num", msg_num_, 10);
        nh_private_.param<std::string>("save_path", save_path_, "/home/dji/output");

        // 验证参数有效性
        if (msg_num_ <= 0) {
            ROS_WARN("Invalid msg_num parameter: %d, setting to 1", msg_num_);
            msg_num_ = 1;
        }
        
        // 输出参数信息
        ROS_INFO("Lidar topic: %s", lidar_topic_.c_str());
        ROS_INFO("Save topic: %s", save_topic_.c_str());
        ROS_INFO("Save path: %s", save_path_.c_str());
        ROS_INFO("Number of messages to collect: %d", msg_num_);
        
        // 初始化状态
        save_flag_ = false;
        msg_count_ = 0;
        collecting_ = false;
        
        // 创建订阅者
        lidar_sub_ = nh_.subscribe(lidar_topic_, 10, &LivoxPointCloudSaver::pcdCallback, this);
        save_sub_ = nh_.subscribe(save_topic_, 10, &LivoxPointCloudSaver::saveFlagCallback, this);
    }
    
    void saveFlagCallback(const std_msgs::Bool::ConstPtr& msg) {
        save_flag_ = msg->data;
        
        // 如果接收到保存信号且未在收集，则重置计数器并开始收集
        if (save_flag_ && !collecting_) {
            cloud_.clear();
            msg_count_ = 0;
            collecting_ = true;
            ROS_INFO("开始收集点云数据...");
        }
    }
    
    void pcdCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        if (collecting_ && msg_count_ < msg_num_) {
            pcl::PointCloud<pcl::PointXYZI> tmp;
            pcl::fromROSMsg(*cloud_msg, tmp);
            
            // 加锁保护共享资源
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            cloud_ += tmp;
            msg_count_++;
            ROS_INFO("已收集 %d/%d 帧点云", msg_count_, msg_num_);
            
            if (msg_count_ >= msg_num_) {
                collecting_ = false;
                ROS_INFO("点云收集完成，等待保存...");
            }
        }
    }
    
    void savePcd() {
        std::lock_guard<std::mutex> lock(cloud_mutex_);

        if (!cloud_.empty()) {
            // 创建带时间戳的文件名
            std::string timestamp = std::to_string(ros::Time::now().toSec());
            std::string pcd_path = save_path_ + "/" + timestamp + ".pcd";

            // 检查并创建保存目录
            try {
                if (!boost::filesystem::exists(save_path_)) {
                    boost::filesystem::create_directories(save_path_);
                    ROS_INFO("创建保存目录: %s", save_path_.c_str());
                }
            } catch (const boost::filesystem::filesystem_error& e) {
                ROS_ERROR("创建目录失败: %s", e.what());
                return;
            }

            // 保存点云
            if (pcl::io::savePCDFileBinary(pcd_path, cloud_) == 0) {
                ROS_INFO("点云已保存至: %s, 点数: %zu", pcd_path.c_str(), cloud_.size());
            } else {
                ROS_ERROR("保存点云失败: %s", pcd_path.c_str());
            }
        } else {
            ROS_WARN("点云为空，无法保存");
        }
    }
    
    void run() {
        ros::Rate loop_rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            
            // 当保存标志为真且收集完成时保存点云
            if (save_flag_ && !collecting_) {
                savePcd();
                save_flag_ = false;
            }
            
            loop_rate.sleep();
        }
    }
};

#endif // LIVOX_CLOUD_SAVER_HPP

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "livox_cloud_saver");
//     ros::NodeHandle nh("~");
    
//     LivoxPointCloudSaver saver(nh);
//     saver.run();
    
//     return 0;
// }