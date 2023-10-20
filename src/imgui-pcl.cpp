// imgui-pcl.cpp : Defines the entry point for the application.
//

#include "imgui-pcl.h"

int main()
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(200, 1);
    for (auto& point : *cloud)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    auto viewer = pcl::visualization::PCLVisualizer("3d Viewer");

    viewer.addPointCloud(cloud);


    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}
