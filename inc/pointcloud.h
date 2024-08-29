#ifndef _POINTCLOUD_H
#define _POINTCLOUD_H
#include <iostream>
#include <k4a/k4a.hpp>
#include "OpencvHead.h"
#include "PCLHead.h"
#include <time.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/console/parse.h>

class lcloud
{
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr source;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_downsampled;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudT;
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_sample;
    pcl::ConditionalRemoval<pcl::PointXYZ> filt;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    // pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normal;
    pcl::PointIndices::Ptr inliers_plane;
    pcl::ModelCoefficients::Ptr coefficients_plane;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    int count;

public:
    lcloud();
    // 通过颜色提取掩膜
    void getMaskAccordingToColor(const cv::Mat &cv_rgbimg, cv::Mat &mask);
    // 获得点云图
    void getXYZPointCloud(const k4a::transformation &k4aTransformation,
                          const k4a::calibration &k4aCalibration,
                          const cv::Mat &cv_depth);

    void getPLY();
    void clearCloud();
};
#endif