cmake_minimum_required(VERSION 2.8.3)

project(hikrobot_camera)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

catkin_package(
 
)

find_package(OpenCV 3.2.0 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
  /opt/MVS/include/
)

LINK_DIRECTORIES(
/opt/MVS/lib/aarch64/
)

add_executable(${PROJECT_NAME}
src/hikrobot_camera.cpp)

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   #GCBase_gcc447_v3_0
   MvCameraControl
   #GenApi_gcc421_v3_0
   MVGigEVisionSDK
   #log4cpp_gcc447_v3_0
   MVRender
   #Log_gcc447_v3_0
   MvUsb3vTL
   #MathParser_gcc447_v3_0
   #NodeMapData_gcc447_v3_0
   MediaProcess
   #XmlParser_gcc447_v3_0
   X11
 )

add_executable(keyboard_listener 
  src/keyboard_listener.cpp
)
target_link_libraries(keyboard_listener
   ${catkin_LIBRARIES}
   
   X11
)
 
add_executable(hikrobot_images_save
  src/hikrobot_images_save.cpp 
)
target_link_libraries(hikrobot_images_save
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}

   X11
)
