#pragma once

#include "stdafx.h"

#include "HoleFilter.h"
#include "FileConvert.h"
#include "Transform3dPoint.h"
#include "PCLDownsampling.h"
#include "Point2Mesher.h"

class EditingUtility {
private:
	HoleFilter hf;
	FileConvert conv;
	Transform3dPoint t3p;
	PCLDownsampling downsampling;
	Point2Mesher p2m;

	std::string calib_path;
	std::string load_path;
	std::string pc_save_path;
	std::string pos_save_path;
	std::string str;
	std::string userName;

	std::vector<std::string> dind104List;
	std::vector<std::string> dind107List;
	std::vector<std::string> dind108List;
	std::vector<std::string> dind109List;
	std::vector<std::string> dind110List;
	std::vector<std::string> dind111List;
	std::vector<std::string> dind112List;

	cv::Point2d focal_length, principal_point;

	inline void split(std::vector<std::string> &v, const std::string &input_string, const std::string &delimiter);
	cv::Point3d getCameraPos(std::string &dind);
	inline pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, cv::Point3d view_pos);
	inline void movingLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
	inline void planeSegmentation(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &plane, pcl::PointCloud<pcl::PointNormal>::Ptr &without_plane);
	inline void euclideanClusterExtraction(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, std::vector<pcl::PointIndices> &cluster);

public:
	EditingUtility(const std::string &dind_id);

	void extractData();
	void createPointCloud();
	void computeCameraPos();
	void frameExtract();
	void loadMatchingFrame();
	void matchingFrameMarge();
	void computeMesh();
};