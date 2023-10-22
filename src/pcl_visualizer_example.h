#pragma once

#include <VtkViewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PclVisualizer {
public:
	PclVisualizer(VtkViewer vtkViewer, pcl::visualization::PCLVisualizer pclviewer);

	void AddPointCloud(pcl::PointCloud < pcl::PointXYZRGBA>::Ptr cloud, std::string name);
	void RenderUI();

private:
	pcl::visualization::PCLVisualizer pclviewer_;
	VtkViewer vtkViewer_{};
};