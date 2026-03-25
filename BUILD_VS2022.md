# 在 Windows 11 + VS2022 中编译构建

## 前置条件

1. 安装 Visual Studio 2022（需包含 "C++ 桌面开发" 工作负载）
2. 安装 CMake（VS2022 自带，或从 https://cmake.org 下载）
3. 安装 vcpkg 包管理器

## 安装 vcpkg

```powershell
git clone https://github.com/microsoft/vcpkg.git C:\vcpkg
cd C:\vcpkg
.\bootstrap-vcpkg.bat
```

设置环境变量：
```powershell
[System.Environment]::SetEnvironmentVariable("VCPKG_ROOT", "C:\vcpkg", "User")
```

## 安装依赖库

```powershell
cd C:\vcpkg
.\vcpkg install ann:x64-windows freeglut:x64-windows glew:x64-windows libpng:x64-windows pngpp:x64-windows cgal:x64-windows
```

## 构建项目

### 方法一：命令行构建

```powershell
cd <项目根目录>
cmake --preset vs2022
cmake --build build --config Release
```

生成的可执行文件位于 `build\Release\TestReconstruct2D.exe`

### 方法二：在 VS2022 中打开

1. 打开 VS2022
2. 选择 "打开本地文件夹"，选择项目根目录
3. VS2022 会自动检测 CMakeLists.txt 和 CMakePresets.json
4. 在顶部工具栏选择 "vs2022" 配置
5. 点击 "生成" -> "全部生成"

## 运行

```powershell
cd build\Release
.\TestReconstruct2D.exe fig1a
```

注意：运行时需要确保 `data` 目录在工作目录下（CMake 构建时会自动复制）。
