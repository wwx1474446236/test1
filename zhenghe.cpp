#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/surface/mls.h>        //��С���˷�ƽ�������ඨ��ͷ�ļ�

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>());
	if (pcl::io::loadPCDFile("E:\\compression.pcd", *cloud) == -1)
	{
		cout << "�������ݶ�ȡʧ�ܣ�" << endl;
	}
	// ���� KD-Tree
	// �Ե����ز��� 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointXYZ> mls_points;

	// ������С����ʵ�ֵĶ���mls
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;

	mls.setComputeNormals(true);  //��������С���˼�������Ҫ���з��߹���

	// Set parameters
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(4);     // ��С���˷������뾶��Խ��ƽ��Ч��Խ�õ��α�Խ��

	// Reconstruct
	mls.process(mls_points);
	cloud_smoothed = mls_points.makeShared();

	// Save output
	pcl::io::savePCDFile("E:\\compression_smooth.pcd", mls_points);

	//��ʼ�Ե�������

	//* the data should be  available in cloud

	// Normal estimation*  �������
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//���÷��߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	tree1->setInputCloud(cloud_smoothed);//��cloud_smoothed����tree����
	n.setInputCloud(cloud_smoothed);//Ϊ���߹��ƶ��������������
	n.setSearchMethod(tree1);//������������
	n.setKSearch(30);//����k�������ص�������Χ
	n.compute(*normals);//���Ʒ��ߴ洢�����normals
	//* normals should not contain the point normals + surface curvatures
	// �������
	std::cout << "normals: " << normals->size() << ", " << "normals fields: " << pcl::getFieldsList(*normals) << std::endl;
	pcl::io::savePCDFileASCII("E:\\compression_normals.pcd", *normals);

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);//
	pcl::concatenateFields(*cloud_smoothed, *normals, *cloud_with_normals);//�����ֶΣ�cloud_with_normals�洢�������
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//��������������
	tree2->setInputCloud(cloud_with_normals);//����������ƹ���tree

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//�������ǻ�����
	pcl::PolygonMesh triangles;//�洢�������ǻ�������ģ��

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(5);         //���������뾶radius����ȷ�����ǻ�ʱkһ�ڽ�����뾶��

	// Set typical values for the parameters
	gp3.setMu(3);                     //���������㵽����������ĳ˻�ϵ�� mu �����ÿ�������������������룬����ʹ���㷨����Ӧ�����ܶȵı仯
	gp3.setMaximumNearestNeighbors(100);//����������������������������Ŀ100 ��
	gp3.setMaximumSurfaceAngle(M_PI / 4);  //45 degrees����������ʱ�����Ƕ� eps_angle ����ĳ�㷨������ڲ�����ķ���ƫ��Ƕȳ��������Ƕ�ʱ������ʱ�Ͳ����Ǹõ㡣
	gp3.setMinimumAngle(M_PI / 18);        //10 degrees���������ǻ��������ε���С�ǣ����� minimum_angle Ϊ��С�ǵ�ֵ��
	gp3.setMaximumAngle(2 * M_PI / 3);    //120 degrees���������ǻ��������ε����ǣ����� maximum_angle Ϊ���ǵ�ֵ��
	gp3.setNormalConsistency(false);     //����һ����־ consistent ������֤���߳���һ�£��������Ϊ true ���ʹ���㷨���ַ��߷���һ�£����Ϊ false �㷨�򲻻���з���һ���Լ�顣

	// Get result
	gp3.setInputCloud(cloud_with_normals);//�����������Ϊ�������
	gp3.setSearchMethod(tree2);           //����������ʽtree2
	gp3.reconstruct(triangles);           //�ؽ���ȡ���ǻ�
   // std::cout << triangles;
	// Additional vertex information
	pcl::io::saveOBJFile("E:\\polygon.obj", triangles);
	std::vector<int> parts = gp3.getPartIDs();//����ؽ���ÿ��� ID, Parts �� 0 ��ʼ��ţ� a-1 ��ʾδ���ӵĵ㡣
	/*
	����ؽ���ÿ���״̬��ȡֵΪ FREE �� FRINGE �� BOUNDARY �� COMPLETED �� NONE ������
	���� NONE ��ʾδ���壬
	FREE ��ʾ�õ�û���� �� �ǻ���������ڣ�Ϊ���ɵ㣬
	COMPLETED ��ʾ�õ������ǻ���������ڣ��������������˵㣬
	BOUNDARY ��ʾ�õ������ǻ�������˱�Ե��
	FRINGE ��ʾ�õ��� �� �ǻ���������ڣ������ӻ�����ص��ߡ�
	*/
	std::vector<int> states = gp3.getPointStates();


	//�������񻯵Ŀ��ӻ�
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);     //���ñ���
	viewer->addPolygonMesh(triangles, "wangge");

	viewer->setRepresentationToSurfaceForAllActors();   //����ģ������Ƭ��ʽ��ʾ
	//viewer->setRepresentationToPointsForAllActors();    //����ģ���Ե���ʽ��ʾ

	//viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();
	// ��ѭ��
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	// Finish
	return (0);
}