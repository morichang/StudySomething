#pragma once

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS 0
#endif

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Kinect.h>
#include "../../../Kinect2Wrapper/Kinect2Wrapper/Kinect2.h"

class PCLUtility {

public:

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr createKinectPointCloud(cv::Size depthSize);

	void convKinectDataToPointCloudXYZRGB(Kinect2Sensor &kinect, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud);

};