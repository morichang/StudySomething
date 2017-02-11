#include "FileConvert.h"

void FileConvert::convPCD2PLY(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	pcl::PCDReader reader;
	pcl::PLYWriter writer;
	std::stringstream FileName;
	
	reader.read<pcl::PointXYZ>(filename + ".pcd", *cloud);

	// PCDファイル形式からPLYファイル形式に変換
	FileName << filename << ".ply";
	std::cout << FileName.str() << std::endl;
	writer.write<pcl::PointXYZ>(FileName.str(), *cloud, true, false);

	return ;
}

void FileConvert::convPLY2PCD(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
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

// DINDで取得したDepth ImageをPoint CloudXYZに変換する
void FileConvert::convPNG2PointCloud(std::string file_path, cv::Mat &depthmap, cv::Point2d param_fl, cv::Point2d param_pp)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ point;
	pcl::PLYWriter ply_writer;

	const int H = depthmap.rows;
	const int W = depthmap.cols;
	const double unitConv = (1.0 / 1000.0);
	const double cx = param_pp.x;
	const double cy = param_pp.y;
	const double recipFx = (1.0 / param_fl.x);
	const double recipFy = (1.0 / param_fl.y);

	// Depth Imageから点群を生成
	#pragma omp parallel for
	for (int v = 0; v < H; ++v){
		#pragma omp parallel for
		for (int u = 0; u < W; ++u){
			// ミリメートルをメートルに変換
			point.z = static_cast<double>(depthmap.at<UINT16>(v, u)) * unitConv;
			point.x = (((double)u - cx) * point.z) * recipFx;
			point.y = (((double)v - cy) * point.z) * recipFy;

			if (point.z != 0.00) {
				pointcloud->push_back(point);
			}
		}
	}

	//filter.Remove_outliers(pointcloud, pointcloud);

	ply_writer.write<pcl::PointXYZ>(file_path + "-pointcloud.ply", *pointcloud, true, false);  // Filename, cloud, binary = true / false, use_camera = true / false

	pointcloud->clear();

	return;
}

// DINDで取得したDepth ImageをPoint CloudRGBに変換する
void FileConvert::convPNG2PointCloudColor(std::string file_path, cv::Mat &colormap, cv::Mat &depthmap, cv::Point2d param_fl, cv::Point2d param_pp)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB point;
	pcl::PLYWriter ply_writer;

	const int H = depthmap.rows;
	const int W = depthmap.cols;

	// Depth Imageから点群を生成
	#pragma omp parallel for
	for (int v = 0; v < H; ++v){
		#pragma omp parallel for
		for (int u = 0; u < W; ++u){
			// ミリメートルをメートルに変換
			point.z = depthmap.at<UINT16>(v, u) / 1000.0;
			point.x = ((double)u - param_pp.x) * point.z / param_fl.x;
			point.y = ((double)v - param_pp.y) * point.z / param_fl.y;

			if (point.z != 0.00) {
				point.r = colormap.at<cv::Vec3b>(v, u)[2];
				point.g = colormap.at<cv::Vec3b>(v, u)[1];
				point.b = colormap.at<cv::Vec3b>(v, u)[0];
				pointcloud->push_back(point);
			}
		}
	}

	ply_writer.write<pcl::PointXYZRGB>(file_path + "pointcloudrgb.ply", *pointcloud, false, false);

	pointcloud->clear();
	depthmap.release();

	return;
}