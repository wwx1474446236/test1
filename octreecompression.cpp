#define _SILENCE_FPOS_SEEKPOS_DEPRECATION_WARNING
#include <iostream>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <string.h>

int main()
{
	pcl::PointCloud<pcl::PointXYZ> sourceCloud;
	pcl::PCDReader reader;
	if (pcl::io::loadPCDFile("E:\\q.pcd", sourceCloud) == -1)//输入点云文件
	{
		PCL_ERROR("Failed to load PCDFile!");
		return -1;
	}
	bool showStatistics = true;
	pcl::io::compression_Profiles_e compressionProfile = pcl::io::MANUAL_CONFIGURATION;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* PointCloudEncoder;
	PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(compressionProfile, showStatistics, 0.2, 1,
		true, 100, true, 8);//输入参数
	std::stringstream compressedData;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>());

	PointCloudEncoder->encodePointCloud(sourceCloud.makeShared(), compressedData);
	compressedData.write("E:\\compressed.bin", sizeof(compressedData));
	PointCloudEncoder->decodePointCloud(compressedData, cloudOut);
	pcl::PCDWriter writer;
	writer.write("E:\\compression.pcd", *cloudOut);


	pcl::visualization::CloudViewer viewer("cloud viewer");
	viewer.showCloud(cloudOut);

	while (!viewer.wasStopped()) {

	}
	system("pause");
	return 0;
}