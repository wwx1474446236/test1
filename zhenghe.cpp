#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/surface/mls.h>        //最小二乘法平滑处理类定义头文件

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>());
	if (pcl::io::loadPCDFile("E:\\compression.pcd", *cloud) == -1)
	{
		cout << "点云数据读取失败！" << endl;
	}
	// 创建 KD-Tree
	// 对点云重采样 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointXYZ> mls_points;

	// 定义最小二乘实现的对象mls
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;

	mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计

	// Set parameters
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(4);     // 最小二乘法搜索半径，越大平滑效果越好但形变越大

	// Reconstruct
	mls.process(mls_points);
	cloud_smoothed = mls_points.makeShared();

	// Save output
	pcl::io::savePCDFile("E:\\compression_smooth.pcd", mls_points);

	//开始对点云网格化

	//* the data should be  available in cloud

	// Normal estimation*  法向估计
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//设置法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
	tree1->setInputCloud(cloud_smoothed);//用cloud_smoothed构造tree对象
	n.setInputCloud(cloud_smoothed);//为法线估计对象设置输入点云
	n.setSearchMethod(tree1);//设置搜索方法
	n.setKSearch(30);//设置k邻域搜素的搜索范围
	n.compute(*normals);//估计法线存储结果到normals
	//* normals should not contain the point normals + surface curvatures
	// 输出法线
	std::cout << "normals: " << normals->size() << ", " << "normals fields: " << pcl::getFieldsList(*normals) << std::endl;
	pcl::io::savePCDFileASCII("E:\\compression_normals.pcd", *normals);

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);//
	pcl::concatenateFields(*cloud_smoothed, *normals, *cloud_with_normals);//连接字段，cloud_with_normals存储有向点云
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//定义搜索树对象
	tree2->setInputCloud(cloud_with_normals);//利用有向点云构造tree

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//定义三角化对象
	pcl::PolygonMesh triangles;//存储最终三角化的网络模型

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(5);         //设置搜索半径radius，来确定三角化时k一邻近的球半径。

	// Set typical values for the parameters
	gp3.setMu(3);                     //设置样本点到最近邻域距离的乘积系数 mu 来获得每个样本点的最大搜索距离，这样使得算法自适应点云密度的变化
	gp3.setMaximumNearestNeighbors(100);//设置样本点最多可以搜索的邻域数目100 。
	gp3.setMaximumSurfaceAngle(M_PI / 4);  //45 degrees，设置连接时的最大角度 eps_angle ，当某点法线相对于采样点的法线偏离角度超过该最大角度时，连接时就不考虑该点。
	gp3.setMinimumAngle(M_PI / 18);        //10 degrees，设置三角化后三角形的最小角，参数 minimum_angle 为最小角的值。
	gp3.setMaximumAngle(2 * M_PI / 3);    //120 degrees，设置三角化后三角形的最大角，参数 maximum_angle 为最大角的值。
	gp3.setNormalConsistency(false);     //设置一个标志 consistent ，来保证法线朝向一致，如果设置为 true 则会使得算法保持法线方向一致，如果为 false 算法则不会进行法线一致性检查。

	// Get result
	gp3.setInputCloud(cloud_with_normals);//设置输入点云为有向点云
	gp3.setSearchMethod(tree2);           //设置搜索方式tree2
	gp3.reconstruct(triangles);           //重建提取三角化
   // std::cout << triangles;
	// Additional vertex information
	pcl::io::saveOBJFile("E:\\polygon.obj", triangles);
	std::vector<int> parts = gp3.getPartIDs();//获得重建后每点的 ID, Parts 从 0 开始编号， a-1 表示未连接的点。
	/*
	获得重建后每点的状态，取值为 FREE 、 FRINGE 、 BOUNDARY 、 COMPLETED 、 NONE 常量，
	其中 NONE 表示未定义，
	FREE 表示该点没有在 三 角化后的拓扑内，为自由点，
	COMPLETED 表示该点在三角化后的拓扑内，并且邻域都有拓扑点，
	BOUNDARY 表示该点在三角化后的拓扑边缘，
	FRINGE 表示该点在 三 角化后的拓扑内，其连接会产生重叠边。
	*/
	std::vector<int> states = gp3.getPointStates();


	//三角网格化的可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);     //设置背景
	viewer->addPolygonMesh(triangles, "wangge");

	viewer->setRepresentationToSurfaceForAllActors();   //网格模型以面片形式显示
	//viewer->setRepresentationToPointsForAllActors();    //网格模型以点形式显示

	//viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();
	// 主循环
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	// Finish
	return (0);
}