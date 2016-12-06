#pragma once

#include "stdafx.h"

class FileConvert{
public:

	void convPCD2PLY(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
	void convPLY2PCD(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

};