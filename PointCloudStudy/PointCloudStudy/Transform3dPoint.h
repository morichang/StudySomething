#pragma once

#include <Eigen/Geometry>

class Transform3dPoint{
public:
	Eigen::Matrix4d convMatToEigenMat(cv::Mat m_in);
	pcl::PointCloud<pcl::PointXYZRGB> convLocalToWorld(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
};