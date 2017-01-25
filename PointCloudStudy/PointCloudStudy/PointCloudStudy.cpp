// PointCloudStudy.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "FileConvert.h"
#include "HoleFilter.h"
#include "Point2Mesher.h"
#include "Transform3dPoint.h"
#include "Delaunay3d.h"
#include "Camera.h"
#include "DxLib.h"

int _tmain(int argc, _TCHAR* argv[])
{
	FileConvert conv;
	Transform3dPoint t3p;
	HoleFilter hf;

	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::string dind_id("DIND109");
	std::string file_path("../../../../../CalibrationParameter/" + dind_id + "-20170109/" + dind_id + "-PC");
	std::string pc_save_path("Data_20170109/" + dind_id);

	cv::Point2d focal_length, principal_point;
	focal_length.x = 1081.37;
	focal_length.y = 1081.37;
	principal_point.x = 719.5;
	principal_point.y = 539.5;

	//conv.convPNG2PointCloud(pc_save_path + "/" + dind_id + "_raw_", focal_length, principal_point);
	///*for (int i = 0; i < 1; i++){
	//	conv.convPNG2PointCloud("raw_", focal_length, principal_point);
	//}*/

	//pcl::io::loadPLYFile(pc_save_path + "/" + dind_id + "_raw_pointcloud.ply", *in_cloud);

	//t3p.convLocalToWorld(file_path, in_cloud, out_cloud);

	//pcl::io::savePLYFile(pc_save_path + "/" + dind_id + "_world_pointcloud.ply", *out_cloud);

	cv::Mat color = cv::imread(pc_save_path + "/" + dind_id + "_raw_color.png", -1);
	cv::Mat depth = cv::imread(pc_save_path + "/" + dind_id + "_raw_depth.png", -1);
	cv::Mat color_lab;
	cv::Mat trunc_color, trunc_depth;
	cv::Mat filtered_depth;
	cv::Mat depth_color;

	int r = 3;
	int d = 2 * r + 1;
	double sc = 50.0;
	double ss = 2.0;

	hf.image_truncate(color, depth, trunc_color, trunc_depth);

	//cv::imwrite(pc_save_path + "/" + dind_id + "_raw_trunc_color.png", trunc_color);
	//cv::imwrite(pc_save_path + "/" + dind_id + "_raw_trunc_depth.png", trunc_depth);

	cv::cvtColor(trunc_color, color_lab, CV_BGR2Lab);
	//hf.BGR2Lab(trunc_color, color_lab, CV_8UC3);

	/*std::cout << (color_lab.type() == CV_8UC3 ? "CV_8UC3" : "Other") << std::endl;
	std::cout << (color_lab.type() == CV_64FC3 ? "CV_64FC3" : "Other") << std::endl;
	std::cout << (depth.type() == CV_16UC1 ? "CV_16UC1" : "Other") << std::endl;*/

	//filtered_depth = trunc_depth.clone();

	//hf.hole_filter(color_lab, trunc_depth, filtered_depth, cv::Size(d, d), sc, ss);

	//filtered_depth.convertTo(depth_color, CV_8U, -255.0f / 8000.0f, 255.0f);
	//cv::applyColorMap(depth_color, depth_color, cv::COLORMAP_JET);
	
	std::cout << "Finish!" << std::endl;

	//cv::imshow("Color Lab", color_lab);
	//cv::imshow("Result", depth_color);

	//cv::imwrite(pc_save_path + "/" + dind_id + "_filterd_trunc_depth.png", filtered_depth);
	//cv::imwrite(pc_save_path + "/" + dind_id + "_filterd_trunc_depth_color.png", depth_color);

	//conv.convPNG2PointCloud(pc_save_path + "/" + dind_id + "_out_", focal_length, principal_point);
	conv.convPNG2PointCloudColor(pc_save_path + "/" + dind_id, focal_length, principal_point);

	//Sleep(INFINITE);
	cv::waitKey();

	return 0;
}