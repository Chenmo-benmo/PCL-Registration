
# 记录每次更新了哪些内容 #

# 20230414版本  
1. 修改了每个模块中的CMakeLists.txt  
2. 删除了ccViewer模块中用到Plugion代码  
3. 编译成功，但拖拉点云文件不能可视化

# 20230418  
1. 添加了PCL库，编译成功


# 20230421
1. 添加了PCL库点云格式转换到CC点云格式的实现pclutils
2. 添加action实现打开pcd文件并可视化

# Chenmobenmo 20241012
1. 在C:\Users\Administrator\Desktop\ccCloudViewer-main下添加了csv2pcd文件，此文件是一个csv转pcd的程序
2. 请将此文件夹放在桌面上
3. 进入prefix文件可以使用VS对程序进行编写

# Chenmobenmo 241024
1. 添加了点云滤波并初步实现滤波效果，但运行时间较长，需要进一步提高效率

# Chenmobenmo 241209
1. ccCloudViewer-main 解压缩之后点击 ccCloudViewer-main\build\Desktop_Qt_5_15_2_MSVC2019_64bit-Debug\CloudViewer.exe 即可使用
2. 删除 ccCloudViewer-main\build 内所有文件
   删除 ccCloudViewer-main\CMakeLists.txt.user 文件
   再使用 Qt 打开 ccCloudViewer-main\CMakeLists.txt 文件
   选择 Qt_5_15_2_MSVC2019_64bit-Debug 构建，构建成功即可使用 Qt 进行开发
3. 删除 ccCloudViewer-main\prefix 内所有文件
   使用 cmake 重新构建，ccCloudViewer-main，ccCloudViewer-main\prefix
   构建成功即可使用 VS2022 进行开发