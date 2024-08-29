#include "../inc/pointcloud.h"
using namespace std;
using namespace pcl;
using namespace cv;
lcloud::lcloud()
{
    source = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();             // 初始化点云对象
    source_downsampled = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); // 初始化下采样后的点云对象
    plane = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();              // 初始化用于储存平面点云的对象
    cloudT = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();             // 初始化转换后的点云对象
    cloud_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();        // 初始化储存点云法线的对象
    inliers_plane = std::make_shared<pcl::PointIndices>();                   // 初始化储存平面点云内点索引的对象
    coefficients_plane = std::make_shared<pcl::ModelCoefficients>();         // 初始化储存平面模型系数的对象
    count = 0;                                                               // 初始化计数器
}
void lcloud::getMaskAccordingToColor(const Mat &cv_rgbimg, Mat &mask)
{
    Mat hsvImage, mask1, mask2;
    cvtColor(cv_rgbimg, hsvImage, COLOR_BGR2HSV);                   // 将bgr转换为hsv图像
    inRange(hsvImage, Scalar(0, 0, 0), Scalar(180, 255, 46), mask); // 从hsvimage获取二值掩膜，范围内像素值设为255，其余设为0
}

// 深度图转点云
void lcloud::getXYZPointCloud(const k4a::transformation &k4aTransformation, const k4a::calibration &k4aCalibration, const Mat &cv_depthimg)
{
    k4a::image depthImage{nullptr};
    k4a::image xyzImage{nullptr};
    k4a::image pointCloud{nullptr};

    // 从设备获取相机分辨率
    int width = k4aCalibration.color_camera_calibration.resolution_width;
    int height = k4aCalibration.color_camera_calibration.resolution_height;

    depthImage = k4a::image::create_from_buffer(K4A_IMAGE_FORMAT_DEPTH16, width, height, width * (int)sizeof(uint16_t), cv_depthimg.data, width * height * 2, nullptr, nullptr); // 使用深度图像数据创建k4a深度图像
    xyzImage = k4aTransformation.depth_image_to_point_cloud(depthImage, K4A_CALIBRATION_TYPE_COLOR);                                                                             // 将深度图像转换为点云
    auto *xyzImageData = (int16_t *)(void *)xyzImage.get_buffer();                                                                                                               // 获取点云数据的指针
    // 遍历所有像素点筛选有效的点云数据
    for (int i = 0; i < width * height; i++)
    {
        if (xyzImageData[3 * i + 2] == 0 || xyzImageData[3 * i + 2] >= 4000) // 要4m内
            continue;
        if (i % 3 != 0) // 要隔3个像素点
            continue;
        PointXYZ point;
        point.x = xyzImageData[3 * i + 0];
        point.y = xyzImageData[3 * i + 1];
        point.z = xyzImageData[3 * i + 2];
        source->points.push_back(point); // 将点添加到点云源中
    }
    // 释放
    pointCloud.reset();
    xyzImage.reset();
    depthImage.reset();
}

void lcloud::getPLY()
{
    Eigen::Vector4f max_pt;   // 点云最大点
    Eigen::Vector4f min_pt;   // 点云最小点
    Eigen::Vector4f centroid; // 点云中心点
    // pcl::PLYWriter writer;    // 保存点云的PLY文件写入器

    ConditionAnd<PointXYZ>::Ptr range_cond(new ConditionAnd<PointXYZ>()); // 点云过滤器条件，所有条件都满足（or为满足一个即可）
    // y坐标条件范围
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 400.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -400.0)));
    // z坐标条件范围
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 400.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, -400.0)));
    // x坐标条件范围
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 400.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -400.0)));

    filt.setCondition(range_cond);
    filt.setKeepOrganized(false);     // 不保持点云的原始结构
    filt.setInputCloud(source);       // 输入点云
    filt.filter(*source_downsampled); // 执行滤波操作，输出点云到source_downsampled

    vg.setInputCloud(source_downsampled);             // 输入点云
    vg.setLeafSize(15, 15, 15);                       // 设置体素格大小
    vg.setMinimumPointsNumberPerVoxel(1);             // 每个体素格至少需要一个点
    vg.filter(*source_downsampled);                   // 下采样
    getMinMax3D(*source_downsampled, min_pt, max_pt); // 获取下采样后点云的最小点和最大点
    // 去离群
    sor.setInputCloud(source_downsampled); // 输入点云
    sor.setMeanK(50);                      // 设置领域点数量
    sor.setStddevMulThresh(1);             // 设置标准差乘数阈值
    sor.filter(*source_downsampled);       // 执行滤波操作
    // 使用RANSAC算法拟合平面模型
    seg.setOptimizeCoefficients(true);                // 优化系数
    seg.setModelType(pcl::SACMODEL_PLANE);            // 设置模型类型为平面
    seg.setMethodType(pcl::SAC_RANSAC);               // 使用RANSAC算法
    seg.setDistanceThreshold(5);                      // 距离阈值
    seg.setMaxIterations(10000);                      // 最大迭代次数
    seg.setInputCloud(source_downsampled);            // 输入点云
    seg.segment(*inliers_plane, *coefficients_plane); // 执行模型拟合
    // 提取平面点云
    extract.setInputCloud(source_downsampled); // 输入点云
    extract.setIndices(inliers_plane);         // 设置要提取的点的索引
    extract.setNegative(false);                // 提取匹配的点
    extract.filter(*plane);                    // 执行提取操作

    if (plane->size() < 20)
    {
        cout << "平面点云数量太少" << endl;
        return;
    }
    if (count == 18) // 如果计数器大于18
    {
        pcl::io::savePLYFile("/home/dzx/下载/test1/testcloud/dy.ply", *plane); // 保存点云到文件
        cout << "保存成功" << endl;
    }
    count++;
}
// 清除
void lcloud::clearCloud()
{
    source->clear();
    source_downsampled->clear();
    cloud_normals->clear();
    plane->clear();
    cloudT->clear();
    inliers_plane->header.frame_id.clear();
    inliers_plane->indices.clear();
    coefficients_plane->header.frame_id.clear();
    coefficients_plane->values.clear();
}