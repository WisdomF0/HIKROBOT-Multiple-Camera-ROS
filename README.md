# HIKROBOT-Multiple-Camera-ROS
可同时驱动多个海康（网口）摄像头，目前实现了两个，需要更多个的可以按照我的例程继续添加

## 说明
1、需要先安装海康相机的SDK驱动，并在MVS.sh中能打开自己的海康摄像头并根据自己的需要更改相机配置，当前实现仅添加了检测相机数量并取图发布ros话题功能  
2、需要在ros实现中通过修改配置文件去修改相机参数可以查看官网API手册添加修改代码：  
file:///opt/MVS/doc/Machine%20Vision%20Camera%20SDK%20(C)_Developer%20Guide_V4.3.0_CH/html/index.html  

## 编译
mkdir catkin_ws  
cd catkin_ws  
mkdir src  
cd src  
git clone
