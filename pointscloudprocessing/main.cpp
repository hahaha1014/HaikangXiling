#include <iostream>
#include <fstream>
#include <algorithm>
#include <stdio.h>

#include<pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include<pcl/filters/passthrough.h>  //ֱͨ�˲���ͷ�ļ�
#include<pcl/filters/voxel_grid.h>  //�����˲���ͷ�ļ�
#include<pcl/filters/statistical_outlier_removal.h>   //ͳ���˲���ͷ�ļ�
#include <pcl/filters/conditional_removal.h>    //�����˲���ͷ�ļ�
#include <pcl/filters/radius_outlier_removal.h>   //�뾶�˲���ͷ�ļ�

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
    vtkObject::GlobalWarningDisplayOff();//����vtk������Ϣ

    ///****************************************************
    /*�����������ݼ���*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    reader.read("1.pcd", *cloud);    //��ȡ���Ƶ�cloud��

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer:pre-process"));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("3D Viewer:normals"));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("3D Viewer:segmentation"));
    viewer->initCameraParameters();//��ʼ���������

    //��һ��������ʾ���ݽ����趨
    int v1(0);
    viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("sample cloud", 10, 10, "sample cloud", v1);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);
    std::cout << "ԭʼ�������ݵ�����" << cloud->points.size() << std::endl;

    ///****************************************************
    /*ֱͨ�˲����Ե��ƽ��д���*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(cloud);//�������
    passthrough.setFilterFieldName("z");//��z����в���
    passthrough.setFilterLimits(500.0, 3000.0);//����ֱͨ�˲���������Χ
    //passthrough.setFilterLimitsNegative(true);//true��ʾ������Χ�ڣ�false��ʾ������Χ��
    passthrough.filter(*cloud_after_PassThrough);//ִ���˲������˽�������� cloud_after_PassThrough

    //�ڶ���������ʾ���ݽ����趨
    int v2(0);
    viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);//����v2�ı�����ɫ  ������ɫ�����÷�Χ���黯��0-1֮��
    viewer->addText("cloud after PassThrough", 10, 10, "cloud after PassThrough", v2);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_after_PassThrough, "cloud_after_PassThrough", v2);
    std::cout << "ֱͨ�˲���������ݵ�����" << cloud_after_PassThrough->points.size() << std::endl;

    ///****************************************************
    /*�����˲���ʵ���²���*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_voxelgrid(new pcl::PointCloud<pcl::PointXYZ>);//
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setInputCloud(cloud_after_PassThrough);//�����������
    voxelgrid.setLeafSize(10.0f, 10.0f, 10.0f);//AABB�����
    voxelgrid.filter(*cloud_after_voxelgrid);

    //������������ʾ���ݽ����趨
    int v3(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v3);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v3);//����v2�ı�����ɫ  ������ɫ�����÷�Χ���黯��0-1֮��
    viewer->addText("cloud after voxelgrid", 10, 10, "cloud after voxelgrid", v3);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_after_voxelgrid, "cloud_after_voxelgrid", v3);
    std::cout << "���ػ����񷽷���������ݵ�����" << cloud_after_voxelgrid->points.size() << std::endl;

    ///****************************************************
    /*ͳ���˲����˲�*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_StatisticalRemoval(new pcl::PointCloud<pcl::PointXYZ>);//
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> Statistical;
    Statistical.setInputCloud(cloud_after_voxelgrid);
    Statistical.setMeanK(100);//ȡƽ��ֵ���ٽ�����
    Statistical.setStddevMulThresh(0.15);// ���ñ�׼����ֵϵ��
    //Statistical.setStddevMulThresh(100);//�ٽ�������Ŀ���ڶ���ʱ�ᱻ����
    Statistical.filter(*cloud_after_StatisticalRemoval);

    //���ĸ�������ʾ���ݽ����趨
    int v4(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v4);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v4);//����v2�ı�����ɫ  ������ɫ�����÷�Χ���黯��0-1֮��
    viewer->addText("cloud after StatisticalRemoval", 10, 10, "cloud after StatisticalRemoval", v4);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_after_StatisticalRemoval, "cloud_after_StatisticalRemoval", v4);
    std::cout << "ͳ�Ʒ����˲���������ݵ�����" << cloud_after_StatisticalRemoval->points.size() << std::endl;


    ///****************************************************
    /*����������*/
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
    // ����int level=2 ��ʾÿn�������һ��������������float scale=100 ��ʾ��������������Ϊ100��
    viewer1->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_after_StatisticalRemoval, normals, 2, 100, "normals");
    viewer1->addCoordinateSystem(1.0);


    // Ϊ��ȡ�㷨��������������һ��KdTree����
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_after_StatisticalRemoval);
    /**
     * ��������Ǵ���һ��PointIndices��vector����vector��vector <int>�а���ʵ�ʵ�������Ϣ��
     * ÿ����⵽�Ĵص�����������������-��ע�⣬cluster_indices��һ��vector�����������⵽�Ĵص�PointIndices��ʵ����
     * ��ˣ�cluster_indices[0]�������ǵ����е�һ�� cluster(��)������������
     *
     * �ӵ�������ȡ�أ���Ⱥ��,������������������ cluster_indices �С�
     */
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(20.0); // �����ٽ������������뾶�������ݲ
    ec.setMinClusterSize(2000);    // ÿ���أ���Ⱥ������С��С
    ec.setMaxClusterSize(250000);  // ÿ���أ���Ⱥ��������С
    ec.setSearchMethod(tree);     // ���õ��������㷨
    ec.setInputCloud(cloud_after_StatisticalRemoval);   // �����������
    ec.extract(cluster_indices);        // ������ȡ���Ĵأ���ÿ��������������ʽ���浽cluster_indices;


    // Ϊ�˴ӵ������������зָ��ÿ���أ�����������ʵ�������
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it != cluster_indices.end(); ++it) {

        // ÿ�δ���һ���µĵ������ݼ������ҽ����е�ǰ�صĵ�д�뵽�������ݼ��С�
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

        Eigen::Vector4f centroid;  //����
        pcl::compute3DCentroid(*cloud_cluster, centroid); // ��������
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