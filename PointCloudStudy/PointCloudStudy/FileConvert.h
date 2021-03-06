#pragma once

#include "stdafx.h"
#include "CloudFilter.h"


class FileConvert {
private:


public:
	
	CloudFilter filter;
	
	void convPCD2PLY(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
	void convPLY2PCD(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
	void convPNG2PointCloud(std::string file_path, cv::Mat &depthmap, cv::Point2d param_fl, cv::Point2d param_pp);
	void convPNG2PointCloudColor(std::string file_path, cv::Mat &colormap, cv::Mat &depthmap, cv::Point2d param_fl, cv::Point2d param_pp);
	                      
};