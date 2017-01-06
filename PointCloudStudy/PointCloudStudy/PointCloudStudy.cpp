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
	int i = 1;

	std::string name = "Data_20161121/dataset_obj/obj_frame";

	cv::imshow("Key Capture", NULL);

	pcl::PolygonMesh::Ptr polygonMesh(new pcl::PolygonMesh());

	pcl::visualization::PCLVisualizer view("MeshMovie");

	view.initCameraParameters();
	view.setBackgroundColor(0, 0, 0);
	view.setCameraPosition(-1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0);
	pcl::io::loadPLYFile(name + std::to_string(i) + ".ply", *polygonMesh);

	while (true){
		view.addPolygonMesh(*polygonMesh, name + std::to_string(i));
		view.spin();
		key = cv::waitKey(1); //ウィンドウの更新

		if (key == 'q'){
			exit(1);
		}
		else if (key == 'n'){
			++i;
			polygonMesh->polygons.clear();
			view.removePolygonMesh(name + std::to_string(i));
			pcl::io::loadPLYFile(name + std::to_string(i) + ".ply", *polygonMesh);
		}
		++i;
		polygonMesh->polygons.clear();
		view.removePolygonMesh(name + std::to_string(i));
		pcl::io::loadPLYFile(name + std::to_string(i) + ".ply", *polygonMesh);
		if (i <= 60){
			i = 1;
		}
	}
	
	view.~PCLVisualizer();

	return 0;
}

