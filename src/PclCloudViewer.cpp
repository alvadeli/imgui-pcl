#include "PclCloudViewer.h"

PclCloudViewer::PclCloudViewer()
{
	vtkSmartPointer<vtkRenderer> renderer = vtkViewer_.getRenderer();
	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkViewer_.getRenderWindow();
	pclViewer_ = std::make_unique<pcl::visualization::PCLVisualizer>(renderer, renderWindow, "pclViewer", false);
}

void PclCloudViewer::RenderUI()
{
	vtkViewer_.render(); // default render size = ImGui::GetContentRegionAvail()
}

void PclCloudViewer::SetCloudPointSize(int pointSize, const std::string& cloudName)
{
	pclViewer_->setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudName);
}

void PclCloudViewer::SetCloudColor(int color[3], const std::string& cloudName)
{
	pclViewer_->setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, color[0], color[1], color[2], cloudName);
}


