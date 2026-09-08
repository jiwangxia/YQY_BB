# YQY 第三方库与编译环境恢复说明

仓库：[jiwangxia/YQY_BB](https://github.com/jiwangxia/YQY_BB)

## 1. 获取第三方库

从仓库的 [Releases](https://github.com/jiwangxia/YQY_BB/releases) 下载 `ThirdPartyLibrary.zip`。

当前压缩包信息：

- 文件大小：约 1.24 GiB（1,329,644,738 字节）
- SHA-256：`AAB1F1AE0732F673CDEAA5DDA9979E2537308277B30CFB0FFD59CC61AD11636`
- 压缩包内包含目录：`ThirdPartyLibrary`
- 压缩包完整性已通过 7-Zip 测试

将压缩包解压到解决方案根目录，也就是包含 `YQY.sln` 的目录。最终目录必须类似：

```text
YQY.sln
ThirdPartyLibrary\
├─ Config_Debug.props
├─ Config_Release.props
├─ cuDSS\
├─ Eigen\
├─ Spectra\
├─ VTK9.4.2\
└─ vcpkg_installed\
```

注意不要解压成多一层目录：

```text
错误：YQY\ThirdPartyLibrary\ThirdPartyLibrary\...
正确：YQY\ThirdPartyLibrary\...
```

## 2. 必须单独安装的软件

压缩包只包含项目使用的第三方库，不包含 CUDA Toolkit、Qt 和 Visual Studio 安装程序。

- Visual Studio 2022：安装“使用 C++ 的桌面开发”、MSVC v143 和 Windows SDK
- Qt 6.8.2 MSVC2022 64-bit：
  `C:\Qt\6.8.2\6.8.2\msvc2022_64`
- CUDA Toolkit 13.0：
  `C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.0`

如果 CUDA 安装在其他位置，可以设置系统环境变量 `CUDA_PATH`，然后重新打开 Visual Studio。

项目会优先使用环境变量和本地 `ThirdPartyLibrary` 路径；不建议把本机的绝对路径写进项目文件。

## 3. 编译运行

1. 双击打开 `YQY.sln`。
2. 平台选择 `x64`。
3. 配置选择 `Release` 或 `Debug`。
4. 选择“生成 → 生成解决方案”。
5. 运行对应程序：

   - Release：`x64\Release\YQY.exe`
   - Debug：`x64\Debug\YQY.exe`

当前 Debug 配置使用 Release 版 Qt/VTK 二进制，以匹配仓库内已有的 VTK Release 库；Debug 仍保留调试符号和调试检查。

## 4. 快速检查关键文件

解压后至少应存在以下文件：

```text
ThirdPartyLibrary\Config_Debug.props
ThirdPartyLibrary\Config_Release.props
ThirdPartyLibrary\cuDSS\...\include\cudss.h
ThirdPartyLibrary\VTK9.4.2\Release\include\vtk-9.4\QVTKOpenGLNativeWidget.h
ThirdPartyLibrary\vcpkg_installed\x64-windows-static-md\include\mkl_pardiso.h
```

如果 Visual Studio 仍提示找不到头文件，先确认目录层级正确，然后关闭并重新打开解决方案，使属性文件重新加载。

## 5. 发布与更新第三方库

第三方库不应直接作为普通 Git 文件提交。更新流程如下：

1. 在本地更新 `ThirdPartyLibrary`。
2. 重新压缩整个 `ThirdPartyLibrary` 目录。
3. 在 GitHub 创建新的 Release，例如 `deps-v1.0.1`。
4. 将新的压缩包作为 Release asset 上传。
5. 更新本文件中的下载说明、文件大小和 SHA-256。

CUDA、cuDSS、Qt、VTK、oneMKL 等组件分别受其供应商许可协议约束。公开发布压缩包前，请确认有权重新分发其中的二进制文件和运行库。
