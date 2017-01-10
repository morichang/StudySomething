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

int _tmain(int argc, _TCHAR* argv[])
{
	FileConvert conv;
	//CloudFillter cf;
	Transform3dPoint t3p;

	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::string dind_id("DIND108");
	std::string file_path("../../../../../CalibrationParameter/" + dind_id + "-20170109/" + dind_id + "-PC");
	std::string pc_save_path("Data_20170109/" + dind_id);

	/*cv::Point2d focal_length, principal_point;
	focal_length.x = 1081.37;
	focal_length.y = 1081.37;
	principal_point.x = 959.5;
	principal_point.y = 539.5;

	conv.convPNG2PointCloud(pc_save_path + "/" + dind_id + "_raw_", focal_length, principal_point);*/
	/*for (int i = 0; i < 1; i++){
		conv.convPNG2PointCloud("raw_", focal_length, principal_point);
	}*/

	pcl::io::loadPLYFile(pc_save_path + "/" + dind_id + "_raw_pointcloud.ply", *in_cloud);

	t3p.convLocalToWorld(file_path, in_cloud, out_cloud);

	pcl::io::savePLYFile(pc_save_path + "/" + dind_id + "_world_pointcloud.ply", *out_cloud);
	
	std::cout << "書き込みm@s!" << std::endl;

	Sleep(INFINITE);

	return 0;
}