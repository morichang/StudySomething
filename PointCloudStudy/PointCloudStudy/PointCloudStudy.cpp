// PointCloudStudy.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "FileConvert.h"
#include "Point2Mesher.h"
#include "CloudFillter.h"
#include "Transform3dPoint.h"
#include "Delaunay3d.h"
#include "Camera.h"
#include "DxLib.h"

void draw(std::string name)
{
	std::string str_line;
	std::ifstream ifs;

	ifs.open(name + ".txt", std::ios::in);

	if (ifs.fail())
	{
		std::cout << "ファイルオープン失敗" << std::endl;
		return;
	}

	VECTOR v[3];
	VERTEX3D vertex[3];

	/*for (int i = 0; i < 3; i++)
	{
	vertex[i].norm = VGet(0.0f, 0.0f, 0.0f);
	vertex[i].dif = GetColorU8(0, 0, 255, 255);
	vertex[i].spc = GetColorU8(0, 0, 0, 0);
	vertex[i].u = 0.0f;
	vertex[i].v = 0.0f;
	vertex[i].su = 0.0f;
	vertex[i].sv = 0.0f;
	}*/

	int i = 0;
	while (std::getline(ifs, str_line))
	{
		sscanf_s(str_line.data(),
			"%f, %f, %f, %f, %f, %f, %f, %f, %f",
			&v[0].x, &v[0].y, &v[0].z, &v[1].x, &v[1].y, &v[1].z, &v[2].x, &v[2].y, &v[2].z);
		/*vertex[0].pos = v[0];
		vertex[1].pos = v[1];
		vertex[2].pos = v[2];
		DrawPolygon3D(vertex, 1, DX_NONE_GRAPH, FALSE);*/

		DrawTriangle3D(v[0], v[2], v[3], GetColor(0, 0, 255), FALSE);

		++i;
	}

	std::cout << "DrawTriangle3D Finish!" << std::endl;
}

int _tmain(int argc, _TCHAR* argv[])
{
	FileConvert conv;
	//CloudFillter cf;
	//Transform3dPoint t3p;
	//Camera camera;

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fillter(new pcl::PointCloud<pcl::PointXYZ>);
	//std::string name = "testPcd"/*"20151203T222733.621941"*/, str_line;
	//std::ifstream ifs;

	//pcl::SACSegmentation<pcl::PointXYZRGB> seg;

	/*SetGraphMode(1920, 1080, 32);

	if (DxLib_Init() == -1)
	{
		return -1;
	}*/

	//pcl::io::loadPCDFile(name + ".pcd", *cloud);

	/*conv.convPCD2PLY(name, cloud);*/

	//SetDrawScreen(DX_SCREEN_BACK);

	//t3p.convLocalToWorld(cloud);

	//pcl::io::savePLYFile(name + "_" + ".ply", *cloud);

	//pcl::io::savePLYFile(name + "_1" + ".ply", cf.Remove_outliers(cloud));

	//pcl::io::loadPLYFile(name + "_1" + ".ply", *cloud_fillter);

	long now = GetNowCount();
	long prev = now;
	int deltaTime;

	//char keys[256] = { 0, };
	//while (ProcessMessage() == 0 && keys[KEY_INPUT_ESCAPE] == false)
	//{
	//	ClearDrawScreen();
	//	//時間
	//	now = GetNowCount();
	//	deltaTime = now - prev;
	//	prev = now;

	//	GetHitKeyStateAll(keys); //キー入力

	//	std::cout << keys << std::endl;

	//	camera.update(deltaTime, keys);

	//	draw(name);

	//	ScreenFlip();
	//}
	//DxLib_End();
	for (int i = 0; i < 61; i++){
		cv::Mat depthmap = cv::imread("Data_20161121/dataset_png/depth" + std::to_string(i) + ".png", -1);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

		for (int v = 0; v < depthmap.rows; v++){
			for (int u = 0; u < depthmap.cols; u++){
				pcl::PointXYZ point;

				point.z = depthmap.at<UINT16>(v, u);
				point.x = (u - 260.048) * point.z / 366.581;
				point.y = (v - 202.228) * point.z / 366.581;
				if ((point.x != 0.0) && (point.y != 0.0) && (point.z != 0)){
					pointcloud->push_back(point);
				}
			}
		}

		pcl::io::savePLYFile("Data_20161121/dataset_pc/morichang" + std::to_string(i) + ".ply", *pointcloud);
		pointcloud->clear();
	}
	
	//pcl::io::savePCDFile("morichang.pcd", *pointcloud);
	std::cout << "書き込みm@s!" << std::endl;

	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read("morichang.pcd", *cloud); // Remember to download the file first!

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points (" << pcl::getFieldsList(*cloud) << ").";

	// Create the filtering object
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(20.0f, 20.0f, 20.0f);
	sor.filter(*cloud_filtered);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

	pcl::PCDWriter writer;
	writer.write("morichang_down.pcd", *cloud_filtered, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

	pcl::PointCloud<pcl::PointXYZ>::Ptr morichang(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile("morichang_down.pcd", *morichang);

	pcl::io::savePLYFile("morichang_down.ply", *morichang);
	std::cout << "書き込みm@s!" << std::endl;

	Sleep(INFINITE);

	return 0;
}