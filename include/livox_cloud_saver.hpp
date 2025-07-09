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
    double start_time_;
    bool collecting_;
    bool use_time_mode_;
    std::mutex cloud_mutex_;  // 添加互斥锁
    
    std::string lidar_topic_;  // 提前声明成员变量
    std::string save_topic_;   // 提前声明成员变量

    std::string timestamp;
    
public:
    LivoxPointCloudSaver(ros::NodeHandle& nh) : nh_(nh), nh_private_("~") {
        // 初始化参数
        nh_private_.param<std::string>("lidar_topic", lidar_topic_, "/livox/lidar");
        nh_private_.param<std::string>("save_topic", save_topic_, "/save_image");
        nh_private_.param<std::string>("save_path", save_path_, "/home/dji/output");
        nh_private_.param<bool>("use_time_mode", use_time_mode_, false);
        nh_private_.param<int>("msg_num", msg_num_, 10);

        // 验证参数有效性
        if (use_time_mode_) {
            if (msg_num_ <= 0) {
                ROS_WARN("Invalid record duration: %d, setting to 5", msg_num_);
                msg_num_ = 5;
            }
            ROS_INFO("Time mode: Will record %d seconds of point cloud data", msg_num_);
        } else {
            if (msg_num_ <= 0) {
                ROS_WARN("Invalid frame count: %d, setting to 10", msg_num_);
                msg_num_ = 10;
            }
            ROS_INFO("Frame mode: Will record %d frames of point cloud data", msg_num_);
        }
        
        // 输出参数信息
        ROS_INFO("Lidar topic: %s", lidar_topic_.c_str());
        ROS_INFO("Save topic: %s", save_topic_.c_str());
        ROS_INFO("Save path: %s", save_path_.c_str());
        ROS_INFO("Number of messages to collect: %d", msg_num_);
        
        // 初始化状态
        save_flag_ = false;
        msg_count_ = 0;
        start_time_ = 0.0;
        collecting_ = false;
        
        // 创建订阅者
        lidar_sub_ = nh_.subscribe(lidar_topic_, 10, &LivoxPointCloudSaver::pcdCallback, this);
        save_sub_ = nh_.subscribe(save_topic_, 10, &LivoxPointCloudSaver::saveFlagCallback, this);
    }
    
    void saveFlagCallback(const std_msgs::Bool::ConstPtr& msg) {
        save_flag_ = msg->data;
        
        // 如果接收到保存信号且未在收集，则重置计数器并开始收集
        if (save_flag_ && !collecting_) {
            timestamp = std::to_string(ros::Time::now().toSec());
            start_time_ = ros::Time::now().toSec();
            cloud_.clear();
            msg_count_ = 0;
            collecting_ = true;

            if (use_time_mode_) {
                ROS_INFO("Starting point cloud recording, target duration: %d seconds", msg_num_);
            } else {
                ROS_INFO("Starting point cloud collection, target frames: %d", msg_num_);
            }
        }
    }
    
    void pcdCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        if (!collecting_) return;

        // 处理点云数据
        pcl::PointCloud<pcl::PointXYZI> tmp;
        pcl::fromROSMsg(*cloud_msg, tmp);
        
        // 加锁保护共享资源
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        cloud_ += tmp;
        msg_count_++;

        // 检查是否达到结束条件
        bool should_stop = false;
        if (use_time_mode_) {
            double current_time = ros::Time::now().toSec();
            double elapsed = current_time - start_time_;
            // 计算整数秒数和剩余秒数
            int elapsed_seconds = static_cast<int>(elapsed);
            double remaining = msg_num_ - elapsed;
            
            // 仅在整数秒时显示进度，最后一帧也显示
            bool is_full_second = (elapsed - elapsed_seconds < 0.01);  // 误差容忍
            bool is_last_frame = (remaining <= 0.1);  // 接近结束时强制显示
            
            if (is_full_second || is_last_frame) {
                ROS_INFO("Time mode: Recorded %d/%d seconds, collected %d frames", elapsed_seconds, msg_num_, msg_count_);
            }
            
            should_stop = (elapsed >= msg_num_);
        } else {
            ROS_INFO("Collected %d/%d point cloud frames", msg_count_, msg_num_);
            should_stop = (msg_count_ >= msg_num_);
        }
        
        if (should_stop) {
            collecting_ = false;
            ROS_INFO("Recording completed, waiting to save...");
        }
    }
    
    void savePcd() {
        std::lock_guard<std::mutex> lock(cloud_mutex_);

        if (!cloud_.empty()) {
            // 创建带时间戳的文件名
            std::string pcd_path = save_path_ + "/" + timestamp + ".pcd";

            // 检查并创建保存目录
            try {
                if (!boost::filesystem::exists(save_path_)) {
                    boost::filesystem::create_directories(save_path_);
                    ROS_INFO("Created save directory: %s", save_path_.c_str());
                }
            } catch (const boost::filesystem::filesystem_error& e) {
                ROS_ERROR("Failed to create directory: %s", e.what());
                return;
            }

            // 保存点云
            if (pcl::io::savePCDFileASCII(pcd_path, cloud_) == 0) {
                if (use_time_mode_) {
                    ROS_INFO("Time mode: Point cloud saved to %s, duration: %d seconds, frames: %d, points: %zu", 
                             pcd_path.c_str(), msg_num_, msg_count_, cloud_.size());
                } else {
                    ROS_INFO("Frame mode: Point cloud saved to %s, frames: %d, points: %zu", 
                             pcd_path.c_str(), msg_count_, cloud_.size());
                }
            } else {
                ROS_ERROR("Failed to save point cloud: %s", pcd_path.c_str());
            }
        } else {
            ROS_WARN("Point cloud is empty, nothing to save");
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