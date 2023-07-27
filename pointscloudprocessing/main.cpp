#include <iostream>
#include <fstream>
#include <algorithm>
#include <stdio.h>

#include<pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include<pcl/filters/passthrough.h>  //直通滤波器头文件
#include<pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include<pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件

#include <pcl/features/normal_3d.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Core>
#include <pcl/common/transforms.h>


using namespace std;

int main(int argc, char** argv)
{
    vtkObject::GlobalWarningDisplayOff();//禁用vtk警告信息

    ///****************************************************
    /*创建点云数据集。*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    reader.read("1.pcd", *cloud);    //读取点云到cloud中

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer:pre-process"));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("3D Viewer:normals"));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("3D Viewer:segmentation"));
    viewer->initCameraParameters();//初始化相机参数

    //第一个窗口显示内容进行设定
    int v1(0);
    viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("sample cloud", 10, 10, "sample cloud", v1);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);
    std::cout << "原始点云数据点数：" << cloud->points.size() << std::endl;

    ///****************************************************
    /*直通滤波器对点云进行处理。*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(cloud);//输入点云
    passthrough.setFilterFieldName("z");//对z轴进行操作
    passthrough.setFilterLimits(500.0, 3000.0);//设置直通滤波器操作范围
    //passthrough.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
    passthrough.filter(*cloud_after_PassThrough);//执行滤波，过滤结果保存在 cloud_after_PassThrough

    //第二个窗口显示内容进行设定
    int v2(0);
    viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);//设置v2的背景颜色  背景颜色的设置范围被归化到0-1之间
    viewer->addText("cloud after PassThrough", 10, 10, "cloud after PassThrough", v2);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_after_PassThrough, "cloud_after_PassThrough", v2);
    std::cout << "直通滤波后点云数据点数：" << cloud_after_PassThrough->points.size() << std::endl;

    ///****************************************************
    /*体素滤波器实现下采样*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_voxelgrid(new pcl::PointCloud<pcl::PointXYZ>);//
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setInputCloud(cloud_after_PassThrough);//输入点云数据
    voxelgrid.setLeafSize(10.0f, 10.0f, 10.0f);//AABB长宽高
    voxelgrid.filter(*cloud_after_voxelgrid);

    //第三个窗口显示内容进行设定
    int v3(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v3);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v3);//设置v2的背景颜色  背景颜色的设置范围被归化到0-1之间
    viewer->addText("cloud after voxelgrid", 10, 10, "cloud after voxelgrid", v3);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_after_voxelgrid, "cloud_after_voxelgrid", v3);
    std::cout << "体素化网格方法后点云数据点数：" << cloud_after_voxelgrid->points.size() << std::endl;

    ///****************************************************
    /*统计滤波器滤波*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_StatisticalRemoval(new pcl::PointCloud<pcl::PointXYZ>);//
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> Statistical;
    Statistical.setInputCloud(cloud_after_voxelgrid);
    Statistical.setMeanK(100);//取平均值的临近点数
    Statistical.setStddevMulThresh(0.15);// 设置标准差阈值系数
    //Statistical.setStddevMulThresh(100);//临近点数数目少于多少时会被舍弃
    Statistical.filter(*cloud_after_StatisticalRemoval);

    //第四个窗口显示内容进行设定
    int v4(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v4);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v4);//设置v2的背景颜色  背景颜色的设置范围被归化到0-1之间
    viewer->addText("cloud after StatisticalRemoval", 10, 10, "cloud after StatisticalRemoval", v4);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_after_StatisticalRemoval, "cloud_after_StatisticalRemoval", v4);
    std::cout << "统计分析滤波后点云数据点数：" << cloud_after_StatisticalRemoval->points.size() << std::endl;


    ///****************************************************
    /*法向量计算*/
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud_after_StatisticalRemoval);
    // For every point, use all neighbors in a radius of 100cm.
    normalEstimation.setRadiusSearch(100);
    // A kd-tree is a data structure that makes searches efficient. More about it later.
    // The normal estimation object will use it to find nearest neighbors.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);

    viewer1->addPointCloud<pcl::PointXYZ>(cloud_after_StatisticalRemoval, "cloud_after_StatisticalRemoval");
    // 参数int level=2 表示每n个点绘制一个法向量，参数float scale=100 表示法向量长度缩放为100倍
    viewer1->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_after_StatisticalRemoval, normals, 2, 100, "normals");
    viewer1->addCoordinateSystem(1.0);


    // 为提取算法的搜索方法创建一个KdTree对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_after_StatisticalRemoval);
    /**
     * 在这里，我们创建一个PointIndices的vector，该vector在vector <int>中包含实际的索引信息。
     * 每个检测到的簇的索引都保存在这里-请注意，cluster_indices是一个vector，包含多个检测到的簇的PointIndices的实例。
     * 因此，cluster_indices[0]包含我们点云中第一个 cluster(簇)的所有索引。
     *
     * 从点云中提取簇（集群）,并将点云索引保存在 cluster_indices 中。
     */
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(20.0); // 设置临近搜索的搜索半径（搜索容差）
    ec.setMinClusterSize(2000);    // 每个簇（集群）的最小大小
    ec.setMaxClusterSize(250000);  // 每个簇（集群）的最大大小
    ec.setSearchMethod(tree);     // 设置点云搜索算法
    ec.setInputCloud(cloud_after_StatisticalRemoval);   // 设置输入点云
    ec.extract(cluster_indices);        // 设置提取到的簇，将每个簇以索引的形式保存到cluster_indices;


    // 为了从点云索引向量中分割出每个簇，必须迭代访问点云索引
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it != cluster_indices.end(); ++it) {

        // 每次创建一个新的点云数据集，并且将所有当前簇的点写入到点云数据集中。
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        const std::vector<int>& indices = it->indices;

        for (std::vector<int>::const_iterator pit = indices.begin(); pit != indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud_after_StatisticalRemoval->points[*pit]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points."
            << std::endl;

        std::stringstream ss;
        ss << "cloud_cluster_" << j;

        Eigen::Vector4f centroid;  //质心
        pcl::compute3DCentroid(*cloud_cluster, centroid); // 计算质心
        std::cout << "Cluster x y z: " << centroid(0) << " " << centroid(1) << " " << centroid(2) << std::endl;

        pcl::PointXYZ center;
        center.x = centroid(0);
        center.y = centroid(1);
        center.z = centroid(2);
        viewer2->addSphere(center, 20, 255, 0, 0, "sphere"+to_string(j));

        // Generate a random (bright) color
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> single_color(cloud_cluster);
        viewer2->addPointCloud<pcl::PointXYZ>(cloud_cluster, single_color, ss.str());
        viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());

        j++;
    }
    std::cout << "cloud size: " << cluster_indices.size() << std::endl;

    while (!viewer->wasStopped()) {
        viewer->spinOnce(1000);
    }

    return (0);

}