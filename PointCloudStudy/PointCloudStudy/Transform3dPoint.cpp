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

void Transform3dPoint::convLocalToWorld(std::string file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud){
	cv::FileStorage cvfs(file_path + "_extrinsic_calc.xml", CV_STORAGE_READ);  //DIND毎にここのパラメータxmlを変える
	cv::FileNode node(cvfs.fs, NULL);
	/*cv::FileNode fn = node[std::string("param3D_array")];

	std::vector<cv::Mat>transformationParameter3D;

	for (int i = 0; i < fn.size(); i++) {
		cv::Mat m;
		cv::read(fn[i], m);
		transformationParameter3D.push_back(m);
	}

	for (auto Rt : transformationParameter3D) {
		auto ER = convMatToEigenMat(Rt);
		pcl::transformPointCloud(*in_cloud, *in_cloud, ER);
	}*/

	//cv::Mat Rt;
	//if (fn.empty()) std::cout << "param3D_array パースエラー" << std::endl;
	//else cv::read(fn, Rt);

	//auto ER = convMatToEigenMat(Rt);
	//pcl::transformPointCloud(*cloud, *cloud, ER);

	cv::Mat R, t;
	if (node["Rot"].empty()) std::cout << "Rot パースエラー" << std::endl;
	else cv::read(node["Rot"], R);

	if (node["trans"].empty()) std::cout << "trans パースエラー" << std::endl;
	else cv::read(node["trans"], t);

	;
	pcl::PointXYZ point;
	out_cloud->clear();
	
	for (auto citr = in_cloud->begin(); citr != in_cloud->end(); citr++) {
		cv::Mat data = (cv::Mat_<double>(3, 1) << (*citr).x, (*citr).y, (*citr).z);
		cv::Mat result = (R.inv() * data) - t;
		point.x = result.at<double>(0);
		point.y = result.at<double>(1);
		point.z = result.at<double>(2);
		out_cloud->push_back(point);
	}

	return ;
}