

List(APPEND imgui_sources
	imgui/imgui.cpp
    imgui/imgui_widgets.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_demo.cpp
    imgui/backends/imgui_impl_glfw.cpp
    imgui/backends/imgui_impl_opengl3.cpp
)

List(APPEND imgui_vtk_sources
    imgui-vtk/VtkViewer.cpp
)

find_package(glfw3 REQUIRED)


add_library(imgui ${imgui_sources} ${imgui_vtk_sources})

find_package(glfw3 CONFIG REQUIRED)
find_package(gl3w CONFIG REQUIRED) 
find_package(VTK COMPONENTS 
  CommonCore
  CommonColor
  CommonDataModel
  FiltersCore
  InteractionStyle
  RenderingCore
  RenderingFreeType
  RenderingGL2PSOpenGL2
  RenderingOpenGL2
  QUIET
)

if (VTK_VERSION VERSION_LESS "9.0.0")
  include(${VTK_USE_FILE})
else()
  # vtk_module_autoinit is needed
  vtk_module_autoinit(
    TARGETS imgui
    MODULES ${VTK_LIBRARIES}
  )
endif()


target_link_libraries(
	imgui
    PUBLIC
	glfw
    unofficial::gl3w::gl3w
    ${VTK_LIBRARIES}
)

target_include_directories(imgui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui-vtk>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )