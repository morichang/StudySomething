// PointCloudStudy.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"

int _tmain(int argc, _TCHAR* argv[])
{
	char key = 0;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	/*pcl::PCDReader reader;
	std::stringstream Filename;
	std::string name = "scene2";

	reader.read<pcl::PointXYZ>("scene2.pcd", *cloud);*/

	//// PCDファイル形式からPLYファイル形式に変換
	/*Filename << name << ".ply";
	std::cout << Filename.str() << std::endl;
	pcl::io::savePLYFileBinary(Filename.str(), *cloud);*/

	// PCDファイルの読み込み
	//pcl::io::loadPCDFile("table_scene_mug_stereo_textured.pcd", *cloud);
	pcl::io::loadPLYFile("xyzrgb_dragon.ply", *cloud);

	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	while (key != 'q'){
		viewer.showCloud(cloud); //テングンちゃんの表示
		key = cv::waitKey(1); //ウィンドウの更新
	}

	return (0);
}

