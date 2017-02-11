#pragma once

#include <Eigen/Geometry>

class Transform3dPoint {
public:
	Eigen::Matrix4d convMatToEigenMat(cv::Mat m_in);
	void convLocalToWorld(std::string file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud);
};