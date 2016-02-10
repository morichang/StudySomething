// PointCloudStudy.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "FileConvert.h"
#include <pcl/octree/octree_impl.h>
#include <pcl/io/io.h>


void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, 	void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	if (event.getKeySym() == "g" && event.keyDown())
	{
		//現在のViewがほしい
	}
	if (event.getKeySym() == "r" && event.keyDown())
	{
		double i = 10.0;
		while (i != 1000.0)
		{
			viewer->setCameraPosition(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0);
			i += 5.0;
		}
		
	}
	if (event.getKeySym() == "f" && event.keyDown())
	{
		viewer->setFullScreen(true);
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	char key = 0;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB> merge;
	//pcl::TextureMesh::Ptr textureMesh(new pcl::TextureMesh());
	pcl::PolygonMesh::Ptr polygonMesh(new pcl::PolygonMesh());
	FileConvert conv;
	//std::string name = "20151203T222732.959635_1";
	//std::string name2 = "20151203T222733.621941_1";
	//std::string name3 = "20151203T222734.016625_1";
	std::string name = "20151203T222732";


	//conv.convPCD2PLY(name, cloud);
	
	//pcl::io::loadPCDFile(name + ".pcd", *cloud);
	//pcl::io::loadPLYFile(name + ".ply", *cloud);
	//pcl::io::loadPLYFile(name2 + ".ply", *cloud2); 
	//pcl::io::loadPLYFile(name3 + ".ply", *cloud3);
	//pcl::io::loadOBJFile(name + ".obj", *polygonMesh);
	//pcl::io::loadOBJFile("merged_mesh.obj", *polygonMesh);
	pcl::io::loadPLYFile("merge_cloud.ply", *merged_cloud);

	/*merge = *cloud;
	merge += *cloud2;
	merge += *cloud3;

	pcl::io::savePLYFile("merge_cloud.ply", merge);*/

	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	//pcl::visualization::PCLVisualizer view("MeshMovie");

	//view.registerMouseCallback();
	//view.initCameraParameters();
	//view.setFullScreen(true);
	//view.setBackgroundColor(0, 0, 0);
	//view.setCameraPosition(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0);
	//view.addPolygonMesh(*polygonMesh);
	//view.addTextureMesh(*textureMesh, "20151203T222732_color");
	while (key != 'q' /*|| !view.wasStopped()*/){
		viewer.showCloud(merged_cloud); //テングンちゃんの表示
		//view.spinOnce(100);
		key = cv::waitKey(1); //ウィンドウの更新
	}
	viewer.wasStopped();
	//view.~PCLVisualizer();

	return 0;
}

