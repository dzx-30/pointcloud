#include "../inc/PCLHead.h"
#include <pcl/surface/mls.h>
#include <iostream>

using namespace std;
using namespace pcl;
typedef PointXYZ PointT;
int main(void)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::visualization::PCLVisualizer viewer("viewer");

    int v1 = 0;                                    // 视口ID
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1); // 视口坐标
    viewer.setBackgroundColor(0, 0, 0, v1);        // 背景颜色黑色

    if (pcl::io::loadPLYFile<pcl::PointXYZ>("/home/dzx/下载/test1/testcloud/dy.ply", *plane) == -1) // 读取ply中点云数据
    {
        PCL_ERROR("Couldn't read file plane.ply \n");
        return -1;
    }

    viewer.addPointCloud(plane, "plane", v1);                                                                    // 将点云添加到视图中
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0, 0.5, "plane", v1); // 设置点云为紫色
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "plane", v1);      // 点云大小为5
    viewer.addCoordinateSystem(50);                                                                              // 添加坐标系，大小为50

    while (!viewer.wasStopped())
    {
        viewer.spin(); // 刷新视图窗口
    }
}