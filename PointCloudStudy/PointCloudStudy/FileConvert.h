#pragma once

#include "stdafx.h"


class FileConvert{
private:


public:
	
	void convPCD2PLY(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
	void convPLY2PCD(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
	void convPNG2PointCloud(std::string file_path, cv::Point2d param_fl, cv::Point2d param_pp);
	                      
};