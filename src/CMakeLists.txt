# CMakeList.txt : CMake project for imgui-pcl, include source and define
# project specific logic here.
#

# Add source to this project's executable.
add_executable (imgui-pcl "imgui-pcl.cpp" "imgui-pcl.h")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET imgui-pcl PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
find_package(PCL CONFIG REQUIRED)

target_link_libraries(
	${PROJECT_NAME} 
	PRIVATE 
	${PCL_LIBRARIES}
)