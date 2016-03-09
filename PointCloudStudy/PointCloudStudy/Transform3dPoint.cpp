#include "Transform3dPoint.h"

Eigen::Matrix4d Transform3dPoint::convMatToEigenMat(cv::Mat m_in){
	Eigen::Matrix4d R;

	//行列を作成する
	if (m_in.cols == 4) {
		R << \
			m_in.at<double>(0, 0), m_in.at<double>(0, 1), m_in.at<double>(0, 2), m_in.at<double>(0, 3), \
			m_in.at<double>(1, 0), m_in.at<double>(1, 1), m_in.at<double>(1, 2), m_in.at<double>(1, 3), \
			m_in.at<double>(2, 0), m_in.at<double>(2, 1), m_in.at<double>(2, 2), m_in.at<double>(2, 3), \
			m_in.at<double>(3, 0), m_in.at<double>(3, 1), m_in.at<double>(3, 2), m_in.at<double>(3, 3);
	}
	else {
		R << \
			m_in.at<double>(0, 0), m_in.at<double>(0, 1), m_in.at<double>(0, 2), 0, \
			m_in.at<double>(1, 0), m_in.at<double>(1, 1), m_in.at<double>(1, 2), 0, \
			m_in.at<double>(2, 0), m_in.at<double>(2, 1), m_in.at<double>(2, 2), 0, \
			0, 0, 0, 1;
	}

	return R;
}

pcl::PointCloud<pcl::PointXYZRGB> Transform3dPoint::convLocalToWorld(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	cv::FileStorage cvfs("receivedParam_DIND105-PC.xml", CV_STORAGE_READ);  //DIND毎にここのパラメータxmlを変える
	cv::FileNode node(cvfs.fs, NULL);
	cv::FileNode fn = node[std::string("param3D_array")];

	std::vector<cv::Mat>transformationParameter3D;

	for (int i = 0; i < fn.size(); i++) {
		cv::Mat m;
		cv::read(fn[i], m);
		transformationParameter3D.push_back(m);
	}

	for (auto Rt : transformationParameter3D){
		auto ER = convMatToEigenMat(Rt);
		pcl::transformPointCloud(*cloud, *cloud, ER);
	}

	return *cloud;
}