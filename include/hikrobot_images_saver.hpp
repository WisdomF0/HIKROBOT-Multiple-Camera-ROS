#ifndef HIKROBOT_SAVE_IMAGES_HPP
#define HIKROBOT_SAVE_IMAGES_HPP

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Bool.h>
#include <string>
#include <boost/filesystem.hpp>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

class HikrobotImageSaver {
public:
    HikrobotImageSaver(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~HikrobotImageSaver() = default;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber save_image_sub_;
    message_filters::Subscriber<sensor_msgs::Image> left_image_sub_;
    message_filters::Subscriber<sensor_msgs::Image> right_image_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>> sync_;
    
    cv::Mat current_frame_left_;
    cv::Mat current_frame_right_;
    bool save_image_flag_;
    std::string save_path_;
    std::string left_topic_;
    std::string right_topic_;
    std::string save_signal_topic_;
    double loop_rate_;

    void saveImageCallback(const std_msgs::Bool::ConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg);
    void saveImages();
    bool validateImage(const cv::Mat& image, const std::string& side) const;
};

HikrobotImageSaver::HikrobotImageSaver(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), save_image_flag_(false) {
    
    // 加载参数
    nh_private_.param<std::string>("save_path", save_path_, "/tmp");
    nh_private_.param<std::string>("left_topic", left_topic_, "/hikrobot_camera_L/image_raw");
    nh_private_.param<std::string>("right_topic", right_topic_, "/hikrobot_camera_R/image_raw");
    nh_private_.param<std::string>("save_signal_topic", save_signal_topic_, "/save_image");
    nh_private_.param<double>("loop_rate", loop_rate_, 10.0);

    ROS_INFO("Image saver initialized with parameters:");
    ROS_INFO("Save Path: %s", save_path_.c_str());
    ROS_INFO("Left Topic: %s", left_topic_.c_str());
    ROS_INFO("Right Topic: %s", right_topic_.c_str());
    ROS_INFO("Save Signal Topic: %s", save_signal_topic_.c_str());
    ROS_INFO("Loop Rate: %.2f Hz", loop_rate_);

    // 创建订阅者
    save_image_sub_ = nh_.subscribe(save_signal_topic_, 1, &HikrobotImageSaver::saveImageCallback, this);
    left_image_sub_.subscribe(nh_, left_topic_, 1);
    right_image_sub_.subscribe(nh_, right_topic_, 1);

    // 初始化同步器
    sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>>(
        left_image_sub_, right_image_sub_, 10);
    sync_->registerCallback(boost::bind(&HikrobotImageSaver::imageCallback, this, _1, _2));

    // 检查保存路径
    if (!boost::filesystem::exists(save_path_)) {
        try {
            boost::filesystem::create_directories(save_path_);
            ROS_INFO("Created save directory: %s", save_path_.c_str());
        } catch (const boost::filesystem::filesystem_error& e) {
            ROS_ERROR("Failed to create directory: %s, Error: %s", save_path_.c_str(), e.what());
            throw;
        }
    }

    // 启动主循环
    ros::Rate rate(loop_rate_);
    while (ros::ok()) {
        ros::spinOnce();
        if (save_image_flag_) {
            saveImages();
            save_image_flag_ = false;
        }
        rate.sleep();
    }
}

void HikrobotImageSaver::saveImageCallback(const std_msgs::Bool::ConstPtr& msg) {
    save_image_flag_ = msg->data;
    if (save_image_flag_) {
        ROS_INFO("Received save image command");
    }
}

void HikrobotImageSaver::imageCallback(const sensor_msgs::ImageConstPtr& left_msg, 
                                     const sensor_msgs::ImageConstPtr& right_msg) {
    try {
        current_frame_left_ = cv_bridge::toCvCopy(left_msg, "bgr8")->image;
        current_frame_right_ = cv_bridge::toCvCopy(right_msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void HikrobotImageSaver::saveImages() {
    if (!validateImage(current_frame_left_, "left") || !validateImage(current_frame_right_, "right")) {
        return;
    }

    std::string timestamp = std::to_string(ros::Time::now().toSec());
    std::string left_image_path = save_path_ + "/left_" + timestamp + ".jpg";
    std::string right_image_path = save_path_ + "/right_" + timestamp + ".jpg";

    // 保存图像并检查结果
    bool left_saved = cv::imwrite(left_image_path, current_frame_left_);
    bool right_saved = cv::imwrite(right_image_path, current_frame_right_);

    if (left_saved && right_saved) {
        ROS_INFO("Images saved successfully: %s, %s", left_image_path.c_str(), right_image_path.c_str());
    } else {
        ROS_ERROR("Failed to save images. Left: %s, Right: %s", 
                 left_saved ? "success" : "failed", 
                 right_saved ? "success" : "failed");
    }
}

bool HikrobotImageSaver::validateImage(const cv::Mat& image, const std::string& side) const {
    if (image.empty()) {
        ROS_WARN("Empty %s image received, cannot save", side.c_str());
        return false;
    }
    return true;
}

#endif // HIKROBOT_SAVE_IMAGES_HPP