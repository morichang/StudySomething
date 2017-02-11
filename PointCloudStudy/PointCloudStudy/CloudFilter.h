#pragma once

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

class CloudFilter {
public:
	void Remove_outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud);
};