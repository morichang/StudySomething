#include "FileConvert.h"

void FileConvert::convPCD2PLY(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
	pcl::PCDReader reader;
	pcl::PLYWriter writer;
	std::stringstream FileName;
	
	reader.read<pcl::PointXYZ>(filename + ".pcd", *cloud);

	// PCDファイル形式からPLYファイル形式に変換
	FileName << filename << ".ply";
	std::cout << FileName.str() << std::endl;
	writer.write<pcl::PointXYZ>(FileName.str(), *cloud, false, false);

	return ;
}

void FileConvert::convPLY2PCD(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
	pcl::PLYReader reader;
	pcl::PCDWriter writer;
	std::stringstream FileName;
	
	reader.read<pcl::PointXYZ>(filename + ".ply", *cloud);

	// PLYファイル形式からPCDファイル形式に変換
	FileName << filename << ".pcd";
	std::cout << FileName.str() << std::endl;
	writer.write<pcl::PointXYZ>(FileName.str(), *cloud, false);

	return ;
}

// DINDで取得したDepth ImageをPoint Cloudに変換する
void FileConvert::convPNG2PointCloud(std::string file_path, cv::Point2d param_fl, cv::Point2d param_pp){
	cv::Mat depthmap = cv::imread(file_path + "depth.png", -1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ point;
	pcl::PLYWriter ply_writer;

	// Depth Imageから点群を生成
	for (int v = 0; v < depthmap.rows; ++v){
		for (int u = 0; u < depthmap.cols; ++u){
			// ミリメートルをメートルに変換
			point.z = depthmap.at<UINT16>(v, u) / 1000.0;
			point.x = ((double)u - param_pp.x) * point.z / param_fl.x;
			point.y = ((double)v - param_pp.y) * point.z / param_fl.y;

			if (point.z != 0.00) {
				pointcloud->push_back(point);
			}
		}
	}

	filter.Remove_outliers(pointcloud, pointcloud);

	// カメラの原点を追加
	point.x = 0.00;
	point.y = 0.00;
	point.z = 0.00;
	pointcloud->push_back(point);

	ply_writer.write<pcl::PointXYZ>(file_path + "pointcloud.ply", *pointcloud, false, false);
	pointcloud->clear();
	depthmap.release();

	return;
}