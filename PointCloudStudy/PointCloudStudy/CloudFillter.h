#pragma once

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

class CloudFillter{
public:
	pcl::PointCloud<pcl::PointXYZRGB> Remove_outliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
};