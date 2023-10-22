#include "PclCloudViewer.h"


template<class T>
void PclCloudViewer::AddPointCloud(std::shared_ptr<pcl::PointCloud<T>>& cloud, 
	const std::string& name)
{
	pclViewer_->addPointCloud<T>(cloud, name);
}

