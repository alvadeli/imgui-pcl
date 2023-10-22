#pragma once

#include <VtkViewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PclCloudViewer{
public:
	PclCloudViewer();


	void RenderUI();

	void SetCloudPointSize(int pointSize, const std::string& name);
	void SetCloudColor(int color[3], const std::string& name);

	void UpdatePointCloud(pcl::PointCloud < pcl::PointXYZRGBA>::Ptr cloud, const std::string& name);

	template<class T>
	void AddPointCloud(std::shared_ptr<pcl::PointCloud<T>>& cloud,
		const std::string& name);

private:
	VtkViewer vtkViewer_;
	std::unique_ptr<pcl::visualization::PCLVisualizer> pclViewer_ = nullptr;
};

#include "PclCloudViewer.hpp"