#pragma once


#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>

#include <GL/gl3w.h>  
#include <GLFW/glfw3.h> 

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <VtkViewer.h>



class MainWindow
{

private:
	bool Init();
	void Update();
	void NewFrame();
	void Render();
	void Shutdown();

	static void glfw_error_callback(int error, const char* description)
	{
		fprintf(stderr, "Glfw Error %d: %s\n", error, description);
	}

private:
	GLFWwindow* window_{};
	VtkViewer vtkViewer;
	pcl::visualization::PCLVisualizer pclViwewer;
};