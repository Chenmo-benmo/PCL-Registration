删除 ccCloudViewer-main\CMakeLists.txt.user 文件
再使用 Qt 打开 ccCloudViewer-main\CMakeLists.txt 文件
选择 Qt_5_15_2_MSVC2019_64bit-Debug 构建，构建成功即可使用 Qt 进行开发
使用 cmake 重新构建，ccCloudViewer-mai，ccCloudViewer-main\prefix，构建成功即可使用 VS2022 进行开发

实现基于PCL的混合滤波算法，将导入点云图像滤除物体外的部分。
开发三视角自动拼接算法，通过对物体的三个不同角度的点云图像进行旋转拼接得到一个完整的物体点云模型。
使用OpenGL展示合成的完整物体点云模型。
