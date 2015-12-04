// PointCloudStudy.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "FileConvert.h"
#include <pcl/octree/octree_impl.h>

int _tmain(int argc, _TCHAR* argv[])
{
	char key = 0;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	FileConvert conv;
	std::string name = "20151203T222733.871536";

	//conv.convPCD2PLY(name, cloud);
	
	//pcl::io::loadPCDFile(name + ".pcd", *cloud);
	pcl::io::loadPLYFile(name + ".ply", *cloud);


	//法線の推定
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	pcl::search::KdTree<pcl::PointNormal>::Ptr searchtree(new pcl::search::KdTree<pcl::PointNormal>);
	searchtree->setInputCloud(cloud_with_normals);

	//物体の初期化
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(searchtree);
	gp3.reconstruct(triangles);

	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	while (key != 'q'){
		viewer.showCloud(cloud); //テングンちゃんの表示
		key = cv::waitKey(1); //ウィンドウの更新
	}

	return 0;
}

