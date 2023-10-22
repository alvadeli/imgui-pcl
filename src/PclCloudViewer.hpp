#include "PclCloudViewer.h"


template<class T>
void PclCloudViewer::AddPointCloud(std::shared_ptr<pcl::PointCloud<T>> cloud,
	const std::string& name)
{
	pclViewer_->addPointCloud(cloud, name);
}

template<class T>
void PclCloudViewer::UpdatePointCloud(std::shared_ptr<pcl::PointCloud<T>> cloud,
	const std::string& name)
{
	pclViewer_->updatePointCloud(cloud, name);
}

