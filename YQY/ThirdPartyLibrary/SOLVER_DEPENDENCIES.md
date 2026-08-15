# 稀疏求解器依赖

## oneMKL

- 版本：2025.2.0
- 安装方式：`vcpkg install intel-mkl:x64-windows-static-md`
- 本机路径：`D:\VS\Vcpkg\vcpkg\installed\x64-windows-static-md`
- 链接方式：LP64、Intel OpenMP、多线程、MKL 静态链接
- 随程序复制：`oneMKLRuntime\libiomp5md.dll`

## cuDSS

- 版本：0.8.0.10
- CUDA 版本：13
- 平台：Windows x64
- 官方压缩包 SHA-256：`66d085917ba2aba0c61dd0cfde0ae284df9813709f60c4cd215c47f8929eb780`
- 项目路径：`cuDSS\libcudss-windows-x86_64-0.8.0.10_cuda13-archive`

发布程序时需要同时保留 NVIDIA cuDSS 许可文件、`cudss64_0.dll` 和 Intel OpenMP 运行库许可。
