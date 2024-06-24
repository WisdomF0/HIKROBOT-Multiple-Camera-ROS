#ifndef CAMERA_HPP
#define CAMERA_HPP
#include "ros/ros.h"
#include <stdio.h>
#include <pthread.h>
#include <thread>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"

namespace camera
{
    std::vector<cv::Mat> frames;
    std::vector<bool> frame_emptys;
    std::vector<pthread_mutex_t> mutexs;
    bool SysteamTime;

    image_transport::CameraPublisher imageL_pub;
    image_transport::CameraPublisher imageR_pub;
    sensor_msgs::Image imageL_msg;
    sensor_msgs::Image imageR_msg;
    sensor_msgs::CameraInfo cameraL_info_msg;
    sensor_msgs::CameraInfo cameraR_info_msg;

    cv_bridge::CvImagePtr cv_ptr_l;
    cv_bridge::CvImagePtr cv_ptr_r;

    ros::Time ConvertToROSTime(uint32_t nDevTimeStampHigh, uint32_t nDevTimeStampLow);

    struct ThreadData {
        int ndevice;
        void* handle;
        MVCC_INTVALUE stParam;
    };

    class Camera
    {
    public:
        Camera(ros::NodeHandle &node);
        ~Camera();

        bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
        void RunCamera(int ndevice, void* &handle); // 初始化和启动相机
        static void* WorkThread(void* p_handle); // 工作线程函数

    private:
        std::vector<void*> handles; // 创建多个相机句柄
        MV_CC_DEVICE_INFO_LIST stDeviceList;  // 枚举设备
        std::vector<pthread_t> threads; // 存储线程ID

        int nRet;
        int TriggerMode;
        // 触发模式：MV_TRIGGER_SOURCE_LINE0 = 0,MV_TRIGGER_SOURCE_LINE1 = 1,MV_TRIGGER_SOURCE_LINE2 = 2,
        // MV_TRIGGER_SOURCE_LINE3 = 3,MV_TRIGGER_SOURCE_COUNTER0 = 4,MV_TRIGGER_SOURCE_SOFTWARE = 7,
        // MV_TRIGGER_SOURCE_FrequencyConverter = 8
        int TriggerSource;
    };

    Camera::Camera(ros::NodeHandle &node)
    {
        //读取待设置的摄像头参数 第三个参数是默认值 yaml文件未给出该值时生效
        node.param("TriggerMode", TriggerMode, 0);    //0为不启用触发，1为启用
        node.param("TriggerSource", TriggerSource, 0);  //设置触发模式
        node.param("SysteamTime", SysteamTime, false);

        image_transport::ImageTransport main_cam_image(node);
        imageL_pub = main_cam_image.advertiseCamera("/hikrobot_camera_L/image_raw", 1000);
        imageR_pub = main_cam_image.advertiseCamera("/hikrobot_camera_R/image_raw", 1000);

        cv_ptr_l = boost::make_shared<cv_bridge::CvImage>();
        cv_ptr_r = boost::make_shared<cv_bridge::CvImage>();
        cv_ptr_l->encoding = sensor_msgs::image_encodings::RGB8;
        cv_ptr_r->encoding = sensor_msgs::image_encodings::RGB8;

        // 枚举设备
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        unsigned int nIndex = 0;
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo);

                void* handle = NULL;
                handles.push_back(handle);

                pthread_mutex_t mutex;
                pthread_mutex_init(&mutex, NULL);  // 初始化互斥锁
                mutexs.push_back(mutex);

                cv::Mat frame;
                frames.push_back(frame);

                bool frame_empty = 0;
                frame_emptys.push_back(frame_empty);
            }
            printf("Find Devices: %d\n", stDeviceList.nDeviceNum);
        }
        else
        {
            printf("Find No Devices!\n");
            exit(-1);
        }

        // 选择设备初始设置并取流
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            RunCamera(i, handles[i]);
        }
    }

    // 初始化和启动相机
    void Camera::RunCamera(int ndevice, void* &handle)
    {
        //选择设备并创建句柄
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[ndevice]);

        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        // 打开设备
        nRet = MV_CC_OpenDevice(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[ndevice]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle,"GevSCPSPacketSize",nPacketSize);
                if(nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
            }
        }

        // 设置触发模式为off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", TriggerMode);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        if(TriggerMode){
            // 设置触发源
            nRet = MV_CC_SetEnumValue(handle, "TriggerSource", TriggerSource);
            if (MV_OK != nRet)
            {
                printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
                exit(-1);
            }
        }

        // ch:获取数据包大小 | en:Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            exit(-1);
        }

        // 开始取流
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        // 创建工作线程
        ThreadData* data = new ThreadData;
        data->ndevice = ndevice;
        data->handle = handle;
        data->stParam = stParam;

        pthread_t nThreadID;
        threads.push_back(nThreadID);
        nRet = pthread_create(&threads[ndevice], NULL, WorkThread, static_cast<void*>(data));
        if (nRet != 0)
        {
            printf("thread create failed. ret = %d\n", nRet);
            exit(-1);
        }
    }

    // 工作线程函数
    void* Camera::WorkThread(void* p_handle)
    {
        int nRet;
        int image_empty_count = 0; //空图帧数
        unsigned char *pDataForRGB = NULL;
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};

        ThreadData* data = static_cast<ThreadData*>(p_handle);
        int ndevice = data->ndevice;
        void* handle = data->handle;
        MVCC_INTVALUE stParam = data->stParam;

        unsigned char * pData = NULL; 
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
        if (NULL == pData)
        {
            printf("pData is null\n");
            exit(-1);
        }
        unsigned int nDataSize = stParam.nCurValue;

        while (ros::ok())
        {
            nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 15);
            if (nRet != MV_OK)
            {
                if (++image_empty_count > 100)
                {
                    ROS_INFO("The Number of Faild Reading Exceed The Set Value!\n");
                    exit(-1);
                }
                continue;
            }
            image_empty_count = 0; //空图帧数

            pDataForRGB = (unsigned char*)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);
            if (NULL == pDataForRGB)
            {
                printf("pDataForRGB is null\n");
                free(pData);  // 释放pData
                exit(-1);
            }

            // 像素格式转换
            // 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
            // 目标像素格式，输出数据缓存，提供的输出缓冲区大小
            stConvertParam.nWidth = stImageInfo.nWidth;
            stConvertParam.nHeight = stImageInfo.nHeight;
            stConvertParam.pSrcData = pData;
            stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
            stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
            stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
            stConvertParam.pDstBuffer = pDataForRGB;
            stConvertParam.nDstBufferSize = stImageInfo.nWidth * stImageInfo.nHeight *  4 + 2048;
            nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);
            if (MV_OK != nRet)
            {
                printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
                free(pData);  // 释放pData
                free(pDataForRGB);  // 释放pDataForRGB
                exit(-1);
            }
            pthread_mutex_lock(&mutexs[ndevice]);
            if(ndevice == 0){
                cv_ptr_l->image = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForRGB).clone();
                imageL_msg = *(cv_ptr_l->toImageMsg());
                if(SysteamTime)
                {
                    imageL_msg.header.stamp = ros::Time::now(); 
                    // ROS_INFO("Using cameraL system time: %f", imageR_msg.header.stamp.toSec());
                }
                else
                {
                    imageL_msg.header.stamp = ConvertToROSTime(stImageInfo.nDevTimeStampHigh, stImageInfo.nDevTimeStampLow);
                    // ROS_INFO("Using cameraL time: %f", imageL_msg.header.stamp.toSec());
                }
                imageL_msg.header.frame_id = "hikrobot_camera";
                cameraL_info_msg.header.frame_id = imageL_msg.header.frame_id;
                cameraL_info_msg.header.stamp = imageL_msg.header.stamp;

                imageL_pub.publish(imageL_msg, cameraL_info_msg);
            }
            else if(ndevice == 1){
                cv_ptr_r->image = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForRGB).clone();
                imageR_msg = *(cv_ptr_r->toImageMsg());
                if(SysteamTime)
                {
                    imageR_msg.header.stamp = ros::Time::now(); 
                    // ROS_INFO("Using cameraR system time: %f", imageR_msg.header.stamp.toSec());
                }
                else
                {
                    imageR_msg.header.stamp = ConvertToROSTime(stImageInfo.nDevTimeStampHigh, stImageInfo.nDevTimeStampLow);
                    // ROS_INFO("Using cameraR time: %f", imageR_msg.header.stamp.toSec());
                }
                imageR_msg.header.frame_id = "hikrobot_camera";
                cameraR_info_msg.header.frame_id = imageL_msg.header.frame_id;
                cameraR_info_msg.header.stamp = imageL_msg.header.stamp;

                imageR_pub.publish(imageR_msg, cameraR_info_msg);
            }
            else{
                printf("目前只写了两个相机，多个相机需要在此处报错添加一些代码。");
                exit(-1);
            }
            pthread_mutex_unlock(&mutexs[ndevice]);

            free(pDataForRGB);  // 释放pDataForRGB
        }
        free(pData);
        delete data;  // 释放data
        return 0;
    }

    bool Camera::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
    {
        if (NULL == pstMVDevInfo)
        {
            printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            printf("%s %x\n", "nCurrentIp:", pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                 // 当前IP
            printf("%s %s\n\n", "chUserDefinedName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName); // 用户定义名
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        }
        else
        {
            printf("Not support.\n");
        }
        return true;
    }

    ros::Time ConvertToROSTime(uint32_t nDevTimeStampHigh, uint32_t nDevTimeStampLow)
    {
        uint64_t timestamp = static_cast<uint64_t>(nDevTimeStampHigh) << 32 | nDevTimeStampLow;
        uint64_t seconds = timestamp / 1000000000UL;
        uint64_t nanoseconds = timestamp % 1000000000UL;
        return ros::Time(seconds, nanoseconds);
    }

    Camera::~Camera()
    {
        // 销毁线程
        for (int i = 0; i < threads.size(); i++)
        {
            pthread_join(threads[i], NULL);
        }

        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            // 停止取流
            nRet = MV_CC_StopGrabbing(handles[i]);
            if (MV_OK != nRet)
            {
                printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
                break;
            }

            // 关闭设备
            nRet = MV_CC_CloseDevice(handles[i]);
            if (MV_OK != nRet)
            {
                printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
                break;
            }

            // 销毁句柄
            nRet = MV_CC_DestroyHandle(handles[i]);
            if (MV_OK != nRet)
            {
                printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
                break;
            }
            handles[i] = NULL;
        }

        // 销毁互斥锁
        for (int i = 0; i < mutexs.size(); i++)
        {
            pthread_mutex_destroy(&mutexs[i]);
        }
    }
}
#endif
