#include "FileConvert.h"

void FileConvert::convPCD2PLY(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
	pcl::PCDReader reader;
	std::stringstream FileName;
	
	reader.read<pcl::PointXYZRGB>(filename + ".pcd", *cloud);

	//// PCDファイル形式からPLYファイル形式に変換
	FileName << filename << ".ply";
	std::cout << FileName.str() << std::endl;
	pcl::io::savePLYFileBinary(FileName.str(), *cloud);

	return ;
}

void FileConvert::convPLY2PCD(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
	pcl::PLYReader reader;
	std::stringstream FileName;
	
	reader.read<pcl::PointXYZRGB>(filename + ".ply", *cloud);

	//// PLYファイル形式からPCDファイル形式に変換
	FileName << filename << ".pcd";
	std::cout << FileName.str() << std::endl;
	pcl::io::savePCDFileBinary(FileName.str(), *cloud);

	return ;
}

// DINDで取得したDepth ImageをPoint Cloudに変換する
void FileConvert::convPNG2PointCloud(std::string file_path, cv::Point2d param_fl, cv::Point2d param_pp){
	cv::Mat depthmap = cv::imread(file_path + "depth.png", -1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (int v = 0; v < depthmap.rows; v++){
		for (int u = 0; u < depthmap.cols; u++){
			pcl::PointXYZ point;

			point.z = depthmap.at<UINT16>(v, u);
			point.x = (u - param_pp.x) * point.z / param_fl.x;
			point.y = (v - param_pp.y) * point.z / param_fl.y;

			if ((point.x != 0.0) && (point.y != 0.0) && (point.z != 0)){
				pointcloud->push_back(point);
			}
		}
	}

	pcl::io::savePLYFile(file_path + "pointcloud.ply", *pointcloud);
	pointcloud->clear();
	depthmap.release();

	return;
}