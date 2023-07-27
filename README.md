# HaikangXiling

开发环境：
1. Visual Studio 2019
2. PCL、Boost
3. 海康RGB-D感知相机MV-EB435i
4. SDK_Mv3dRgbd_V1.0.0_Win
  
在Windows平台进行开发， 整个项目由两个部分组成：使用海康RGB-D感知相机MV-EB435i采集三维点云数据，存储成pcd格式点云文件；通过PCL对存储的点云文件进行简单的点云处理，包括直通滤波器滤波，体素滤波器下采样，统计滤波器滤波，法向量计算，点云分割，最终输出点云簇的质心位置，得到视野中无序物品的抓取点位。
