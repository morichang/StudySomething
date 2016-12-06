#pragma once

#include "stdafx.h"


class FileConvert{
private:
	
	cv::Point focalLength;
	cv::Point principalPoint;


public:
	
	void convPCD2PLY(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
	void convPLY2PCD(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
	void convPNG2PointCloud(std::string filename, cv::Point param_fl, cv::Point param_pp);

};