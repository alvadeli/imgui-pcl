#include "pcl_visualizer_example.h"

PclVisualizer::PclVisualizer(VtkViewer vtkViewer, pcl::visualization::PCLVisualizer pclviewer)
{
	vtkViewer_ = vtkViewer;
	pclviewer_ = pclviewer;
}

void PclVisualizer::AddPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::string name)
{
	pclviewer_.addPointCloud(cloud, name);
	pclviewer_.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
}

void PclVisualizer::RenderUI()
{
	ImGui::SetNextWindowSize(ImVec2(360, 240), ImGuiCond_FirstUseEver);
	ImGui::Begin("PointVloud Viewer", nullptr, VtkViewer::NoScrollFlags());

	vtkViewer_.render(); // default render size = ImGui::GetContentRegionAvail()
	ImGui::End();
}
