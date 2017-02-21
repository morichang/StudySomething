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
		auto Pos = viewer->getViewerPose();//現在のViewがほしい
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
	int i = 2;

	std::string name = "C:/Users/Morita/Desktop/marge/Mesh/Frame__";

	cv::imshow("Key Capture", NULL);

	pcl::PolygonMesh::Ptr polygonMesh(new pcl::PolygonMesh());
	pcl::io::loadPLYFile(name + "40-maregemesh.ply", *polygonMesh);

	pcl::visualization::PCLVisualizer view("MeshMovie");

	//view.initCameraParameters();
	view.setBackgroundColor(0, 0, 0);
	view.setCameraPosition(0.0, 2.0, 0.0, 1.0, 1.0, 1.0, 0);
	view.addPolygonMesh(*polygonMesh, "polygon");

	while (true){
		//pcl::io::loadPLYFile(name + std::to_string(i) + "-maregemesh.ply", *polygonMesh);
		//view.addPolygonMesh(*polygonMesh, name + std::to_string(i));
		view.spin();
		
		if (key == 'q'){
			exit(1);
		}
		else if (key == 'n'){
			++i;
			polygonMesh->polygons.clear();
			view.removePolygonMesh(name + std::to_string(i));
		}
		++i;
		//polygonMesh->polygons.clear();
		//view.removePolygonMesh(name + std::to_string(i));

		if (i <= 10){
			i = 1;
		}
		key = cv::waitKey(33); //ウィンドウの更新
	}
	
	view.~PCLVisualizer();

	return 0;
}

