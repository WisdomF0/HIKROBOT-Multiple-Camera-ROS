#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <csignal>  // 添加信号处理头文件

// 保存原始终端设置
struct termios original_termios;

// 设置终端为原始模式
void setTerminalToRaw() {
    tcgetattr(STDIN_FILENO, &original_termios);
    struct termios new_termios = original_termios;
    
    // 禁用规范模式和回显
    new_termios.c_lflag &= ~(ICANON | ECHO);
    // 设置最小读取字符数为1
    new_termios.c_cc[VMIN] = 1;
    // 设置超时为0
    new_termios.c_cc[VTIME] = 0;
    
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
}

// 恢复终端设置
void restoreTerminal() {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
}

// 按键监听线程
void keyboardListener(ros::Publisher& save_pub) {
    // 设置终端为原始模式
    setTerminalToRaw();
    
    std::cout << "Press 's' to save images, or 'q' to quit." << std::endl;
    
    char c;
    while (ros::ok()) {
        // 直接读取一个字符
        if (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == 's' || c == 'S') {
                std_msgs::Bool save_signal;
                save_signal.data = true;
                save_pub.publish(save_signal);
                ROS_INFO("Save signal published.");
            } else if (c == 'q' || c == 'Q') {
                ROS_INFO("Exiting keyboard listener...");
                ros::shutdown();
                break;
            } else {
                ROS_WARN("Unknown command: %c", c);
            }
        }
    }
    
    // 恢复终端设置
    restoreTerminal();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_listener");
    ros::NodeHandle nh;

    // 使用全局命名空间的 signal 函数
    ::signal(SIGINT, [](int signum) {
        restoreTerminal();
        ros::shutdown();
    });

    // 定义保存信号发布者
    ros::Publisher save_pub = nh.advertise<std_msgs::Bool>("/save_image", 1);

    // 启动按键监听线程
    std::thread listener_thread(keyboardListener, std::ref(save_pub));

    // 主线程进入ROS轮询状态
    ros::spin();

    // 等待监听线程结束
    if (listener_thread.joinable()) {
        listener_thread.join();
    }

    return 0;
}