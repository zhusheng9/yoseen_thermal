#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <YoseenSDK/YoseenSDK.h>
#include <string.h>

static ros::Publisher image_pub;
static ros::Publisher temp_pub;

struct ShellView {
    s32 userHandle;                           //用户句柄
    s32 previewHandle;                        //预览句柄
    YoseenLoginInfo loginInfo;                //登入信息
    CameraBasicInfo cameraBasicInfo;          //热像仪基本信息
    YoseenPreviewInfo previewInfo;            //预览信息
};
static ShellView _shellView = { };
static TempFrameFile tempFrameFile = { };

// 伪彩处理函数（温度 -> 彩色图）
static void tempToPseudoColor(const s16* tempData, int width, int height, cv::Mat& colorImg) {
    cv::Mat tempMat(height, width, CV_16SC1, (void*)tempData);
    // cv::Mat norm;
    // tempMat.convertTo(norm, CV_8UC1, 255.0 / 1000);
    // applyColorMap(norm, colorImg, cv::COLORMAP_JET);
    double minVal, maxVal;
    cv::minMaxLoc(tempMat, &minVal, &maxVal);

    // 防止 max = min 导致归一化失败
    if (maxVal - minVal < 1e-3) {
        maxVal = minVal + 1.0;
    }

    cv::Mat norm;
    tempMat.convertTo(norm, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
    applyColorMap(norm, colorImg, cv::COLORMAP_JET);
}

static void __stdcall previewCallback(s32 errorCode, DataFrame* frame, void* customData) {
    if (errorCode != 0 || frame == nullptr) return;

    DataFrameHeader* tempHead = (DataFrameHeader*)frame->Head;
    s16* tempData = (s16*)frame->Temp;

    int width = tempHead->Width;
    int height = tempHead->Height;

    u16 Slope = tempHead->Slope;
	s16 Offset = tempHead->Offset;

    cv::Mat colorImg;
    tempToPseudoColor(tempData, width, height, colorImg);
    
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(header, "bgr8", colorImg).toImageMsg();
    image_pub.publish(imgMsg);

    std_msgs::Float32MultiArray tempMsg;
    tempMsg.layout.dim.resize(2);
    tempMsg.layout.dim[0].label = "height";
    tempMsg.layout.dim[0].size = height;
    tempMsg.layout.dim[0].stride = width * height;
    tempMsg.layout.dim[1].label = "width";
    tempMsg.layout.dim[1].size = width;
    tempMsg.layout.dim[1].stride = width;

    tempMsg.data.resize(width * height);
    for (int i = 0; i < width * height; ++i) {
        tempMsg.data[i] = float(tempData[i]) / Slope + Offset; // 温度浮点值 = 温度整数值 / Slope + Offset
        // tempMsg.data[i] = tempData[i] / 20.0f;
    }
    temp_pub.publish(tempMsg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "yoseen_thermal_node");
    ros::NodeHandle nh;

    image_pub = nh.advertise<sensor_msgs::Image>("/thermal/image", 1);
    temp_pub  = nh.advertise<std_msgs::Float32MultiArray>("/thermal/temperature", 1);

    // 初始化 SDK
    Yoseen_InitSDK();

    // 登录热像仪
    CameraBasicInfo& basicInfo = _shellView.cameraBasicInfo;
    YoseenLoginInfo& loginInfo = _shellView.loginInfo;
    std::string ip;
    nh.param<std::string>("camera_ip", ip, "192.168.1.201");
    strncpy(loginInfo.CameraAddr, ip.c_str(), 31);

    s32 userHandle = Yoseen_Login(&loginInfo, &basicInfo);
    if (userHandle < 0) {
        ROS_ERROR("Failed to login Yoseen device.");
        return -1;
    }
    _shellView.userHandle = userHandle;

    // 开启预览
    YoseenPreviewInfo& previewInfo = _shellView.previewInfo;
    previewInfo.DataType = xxxdatatype_temp;  // 温度流
    previewInfo.CustomCallback = previewCallback;
    previewInfo.CustomData = nullptr;

    s32 previewHandle = Yoseen_StartPreview(userHandle, &previewInfo);
    if (previewHandle < 0) {
        ROS_ERROR("Failed to start preview.");
        return -1;
    }
    _shellView.previewHandle = previewHandle;

    ROS_INFO("Yoseen thermal streaming started...");
    ros::spin();

    // 清理资源
    if (_shellView.previewHandle >= 0) {
        Yoseen_StopPreview(_shellView.previewHandle);
    }

    if (_shellView.userHandle >= 0) {
        Yoseen_Logout(_shellView.userHandle);
    }
    Yoseen_FreeSDK();

    return 0;
}
