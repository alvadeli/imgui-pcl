# CMakeList.txt : CMake project for imgui-pcl, include source and define
# project specific logic here.
#



set(CMAKE_INCLUDE_CURRENT_DIR ON)



# Add source to this project's executable.
add_executable (imgui-pcl "main.cpp"    "PclCloudViewer.h" "PclCloudViewer.cpp" )

#if (CMAKE_VERSION VERSION_GREATER 3.12)
#  set_property(TARGET imgui-pcl PROPERTY CXX_STANDARD 20)
#endif()

# TODO: Add tests and install targets if needed.
find_package(PCL CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(gl3w CONFIG REQUIRED) 

target_link_libraries(
	${PROJECT_NAME} 
	PRIVATE 
	${PCL_LIBRARIES}
	glfw
	imgui
	unofficial::gl3w::gl3w
)