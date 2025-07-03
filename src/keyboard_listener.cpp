#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <thread>

// 按键监听线程
void keyboardListener(ros::Publisher& save_pub) {
    std::string input;
    while (ros::ok()) {
        std::cout << "Press 's' and Enter to save images, or 'q' to quit: ";
        std::cin >> input; // 从标准输入获取按键
        if (input == "s") {
            std_msgs::Bool save_signal;
            save_signal.data = true;
            save_pub.publish(save_signal); // 发布保存信号
            ROS_INFO("Save signal published.");
        } else if (input == "q") {
            ROS_INFO("Exiting keyboard listener...");
            ros::shutdown(); // 退出节点
            break;
        } else {
            ROS_WARN("Unknown command: %s", input.c_str());
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_listener");
    ros::NodeHandle nh;

    // 定义保存信号发布者
    ros::Publisher save_pub = nh.advertise<std_msgs::Bool>("/save_image", 1);

    // 启动按键监听线程
    std::thread listener_thread(keyboardListener, std::ref(save_pub));

    // 主线程进入 ROS spin 状态
    ros::spin();

    // 等待监听线程结束
    if (listener_thread.joinable()) {
        listener_thread.join();
    }

    return 0;
}
