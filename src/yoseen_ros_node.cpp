#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <YoseenSDK/YoseenSDK.h>
#include <string.h>

#include <fcntl.h>        // open, O_RDWR
#include <unistd.h>       // close, read, write
#include <sys/mman.h>     // mmap, PROT_READ, PROT_WRITE, MAP_SHARED, MAP_FAILED
#include <sys/stat.h>     // optional: fstat
#include <sys/types.h>    // optional: off_t, size_t

static ros::Publisher image_pub;
static ros::Publisher temp_pub;

static sensor_msgs::ImagePtr latest_img_msg;
static std_msgs::Float32MultiArray latest_temp_msg;
static bool new_data_available = false;

struct time_stamp {
  int64_t high;
  int64_t low;
};
time_stamp *pointt;

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

    ros::Time rcv_time;
    if (pointt != MAP_FAILED && pointt->low != 0) {
        int64_t b = pointt->low;
        double time_pc = b / 1000000000.0;
        rcv_time = ros::Time(time_pc);
    } else {
        rcv_time = ros::Time::now();
    }

    DataFrameHeader* tempHead = (DataFrameHeader*)frame->Head;
    s16* tempData = (s16*)frame->Temp;

    int width = tempHead->Width;
    int height = tempHead->Height;

    u16 Slope = tempHead->Slope;
	s16 Offset = tempHead->Offset;

    cv::Mat colorImg;
    tempToPseudoColor(tempData, width, height, colorImg);
    
    std_msgs::Header header;
    header.stamp = rcv_time;

    latest_img_msg = cv_bridge::CvImage(header, "bgr8", colorImg).toImageMsg();

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
    latest_temp_msg = tempMsg;
    new_data_available = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "yoseen_thermal_node");
    ros::NodeHandle nh;

    image_pub = nh.advertise<sensor_msgs::Image>("/thermal/image", 1);
    temp_pub  = nh.advertise<std_msgs::Float32MultiArray>("/thermal/temperature", 1);

    const char *user_name = getlogin();
    std::string path_for_time_stamp = "/home/" + std::string(user_name) + "/timeshare";
    const char *shared_file_name = path_for_time_stamp.c_str();
    int fd = open(shared_file_name, O_RDWR);
    pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

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
    ros::Rate loop_rate(10);  // 10Hz 发布频率

    while (ros::ok()) {
        if (new_data_available) {
            // 发布最新的图像和温度数据
            image_pub.publish(latest_img_msg);
            temp_pub.publish(latest_temp_msg);
            new_data_available = false;  // 重置标志位
        }

        ros::spinOnce();  // 处理回调队列
        loop_rate.sleep();  // 等待下一次循环
    }


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
