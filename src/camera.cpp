#include "../inc/camera.h"
#include "../inc/pointcloud.h"

using namespace std;
using namespace k4a;
using namespace cv;

k4a::device device;
lcloud *cloud = new lcloud;

// 设置相机初始配置
void init_kinect(uint32_t &device_count, k4a::device &device, k4a_device_configuration_t &init, capture &capture)
{
    device_count = device::get_installed_count(); // 获取已安装的设备数量
    if (device_count == 0)
    {
        cout << "Error:no K4A devices found." << endl;
        return;
    }
    else
    {
        cout << "Found" << device_count << "connected devices." << endl;
    }

    device = device::open(K4A_DEVICE_DEFAULT); // 打开默认设备
    cout << "Done:open device." << endl;

    init = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;         // 初始化设备配置（所有配置为false）
    init.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; // 设置颜色图像格式为四通道BGRA
    init.color_resolution = K4A_COLOR_RESOLUTION_720P; // 分辨率720P
    init.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;    // 深度模式为NFOV（窄）且未经过下采样
    init.camera_fps = K4A_FRAMES_PER_SECOND_30;        // 帧率为每秒30帧
    init.synchronized_images_only = true;              // 只支持同步图像

    device.start_cameras(&init); // 启用相机（使用init中参数初始化相机）
    cout << "Done:start camera." << endl;
    device.start_imu(); // 启用imu（惯性测量单元，用于捕捉加速度和角速度）
    cout << "Done:start imu." << endl;

    int iAuto = 0; // 计数为0
    while (1)
    {
        if (device.get_capture(&capture))
            cout << iAuto << ". Capture several frames to give auto-exposure" << endl;
        if (iAuto < 30) // 30内自动曝光
        {
            iAuto++;
            continue;
        }
        else
        {
            cout << "Done: auto-exposure" << endl;
            break;
        }
    }
}

int main(int argc, char const *argv[])
{
    k4a::device device;
    uint32_t device_count;
    k4a_device_configuration_t init;

    image k4a_color;
    image k4a_depth;
    image k4a_infrared; // 红外
    image k4a_tf_depth; // 经处理的深度
    capture capture;

    init_kinect(device_count, device, init, capture); // 调用配置设备的函数

    calibration k4aCalibration = device.get_calibration(init.depth_mode, init.color_resolution); // 从设备获取校准数据，传入深度模式和颜色分辨率，得到校准对象
    transformation k4aTransformation = k4a::transformation(k4aCalibration);                      // 使用校准数据创建一个转换对象，用于处理设备捕捉到的图像数据

    Mat cv_color1, cv_depth, cv_infrared, mask, cv_depth_with_mask, cv_color; // 声明mat对象（用于储存处理后的图像信息）

    namedWindow("mask"); // 创建一个名为mask的窗口（用于显示掩模图像）

    clock_t start, end; // 声明计时开始和结束

    while (true) // 捕捉处理图像
    {
        lcloud lcloud;
        if (device.get_capture(&capture, std::chrono::milliseconds(100))) // 如果在100毫秒内捕捉到图像
        {
            k4a_color = capture.get_color_image();                                   // 捕捉彩色图像
            k4a_depth = capture.get_depth_image();                                   // 捕捉深度图像
            k4a_tf_depth = k4aTransformation.depth_image_to_color_camera(k4a_depth); // 处理深度图像

            cv_color = Mat(k4a_color.get_height_pixels(), k4a_color.get_width_pixels(), CV_8UC4, k4a_color.get_buffer());         // 将k4a彩色图转换为opencv的mat对象
            cv_depth = Mat(k4a_tf_depth.get_height_pixels(), k4a_tf_depth.get_width_pixels(), CV_16U, k4a_tf_depth.get_buffer()); // 将k4a深度图转换为opencv的mat对象

            cvtColor(cv_color, cv_color, cv::COLOR_BGRA2BGR); // 将BGRA转换为BGR

            cv_depth.convertTo(cv_depth, CV_8U, 1); // 将深度图像（16位或32位）转换为8位无符号整数格式，乘1表示不改变原值

            cloud->getMaskAccordingToColor(cv_color, mask); // 从彩色图像获取掩膜
            cv_depth.copyTo(cv_depth_with_mask, mask);      // 将掩膜应用到深度图像

            cloud->getXYZPointCloud(k4aTransformation, k4aCalibration, cv_depth_with_mask); // 从处理后的深度图像提取点云数集
            cloud->getPLY();                                                                // 将点云数集保存至PLY文件
            cloud->clearCloud();                                                            // 清除点云数集

            // 测试
            imshow("mask", cv_depth_with_mask); // 显示掩膜后的深度图像
            imshow("color", cv_color);          // 显示彩色窗口
            imshow("depth", cv_depth);          // 显示深度窗口

            // 释放空间
            k4a_color.reset();
            k4a_depth.reset();
            k4a_tf_depth.reset();
            capture.reset();
            cv_color.release();
            cv_depth.release();
            mask.release();
            cv_depth_with_mask.release();

            capture.reset();
            if (cv::waitKey(1) == 27) // 键码27（Esc），退出循环
                break;
        }
    }
    cv::destroyAllWindows();
    device.close();
    return 0;
}