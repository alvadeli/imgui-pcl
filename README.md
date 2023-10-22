# imgui-pcl

![](imgui-pcl-demo.gif)

## vcpkg

- https://github.com/microsoft/vcpkg
- windows: VCPKG_DEFAULT_TRIPLET=x64-windows

### Dependencies

- Point Cloud Library
  - vcpkg install pcl[core,vtk]
- gl3w
- glfw
- vtk
  - will be installed with pcl
