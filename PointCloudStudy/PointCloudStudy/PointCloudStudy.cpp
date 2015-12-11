// PointCloudStudy.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "FileConvert.h"
#include <Eigen/Geometry>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/surface/mls.h>

inline Eigen::Matrix4d convMatToEigenMat(cv::Mat m_in){
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

pcl::PointCloud<pcl::PointXYZRGB> Remove_outliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);//外れ値を除去する点群を入力
	sor.setMeanK(50);//MeanKを設定
	sor.setStddevMulThresh(0.1);
	sor.setNegative(false);//外れ値を出力する場合はtrueにする
	sor.filter(*cloud_filtered);//出力
	return *cloud_filtered;
}

int _tmain(int argc, _TCHAR* argv[])
{
	char key = 0;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fillter(new pcl::PointCloud<pcl::PointXYZRGB>);
	FileConvert conv;
	std::string name = "20151203T222733.621941";

	//conv.convPCD2PLY(name, cloud);
	
	//pcl::io::loadPCDFile(name + ".pcd", *cloud);
	pcl::io::loadPLYFile(name + ".ply", *cloud);

	cv::FileStorage cvfs("receivedParam_DIND105-PC.xml", CV_STORAGE_READ);
	cv::FileNode node(cvfs.fs, NULL);
	cv::FileNode fn = node[std::string("param3D_array")];

	std::vector<cv::Mat>transformationParameter3D;

	for(int i = 0; i < fn.size(); i++) {
		cv::Mat m;
		cv::read(fn[i], m);
		transformationParameter3D.push_back(m);
	}

	std::cout << transformationParameter3D.size() << std::endl;

	//cv::Mat p = (cv::Mat_< double>(4, 1) << 0, 0, 0, 1);
	//cv::Mat result = p;
	for (auto Rt : transformationParameter3D){
		//result = Rt * result;
		auto ER = convMatToEigenMat(Rt);
		pcl::transformPointCloud(*cloud, *cloud, ER);
	}

	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

	//pcl::PointCloud<pcl::PointNormal> mls_points;

	//pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;

	//mls.setComputeNormals(true);

	//// Set parameters
	//mls.setInputCloud(cloud);
	//mls.setPolynomialFit(true);
	//mls.setSearchMethod(tree);
	//mls.setSearchRadius(0.03);

	//// Reconstruct
	//mls.process(mls_points);

	pcl::io::savePLYFile(name + "_" + ".ply", *cloud);

	pcl::io::savePLYFile(name + "_1" + ".ply", Remove_outliers(cloud));

	/*pcl::io::savePLYFile(name + "_2" + ".ply", mls_points);*/

	pcl::io::loadPLYFile(name + "_1" + ".ply", *cloud_fillter);

	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	while (key != 'q'){
		viewer.showCloud(cloud_fillter); //テングンちゃんの表示
		key = cv::waitKey(1); //ウィンドウの更新
	}

	Sleep(INFINITE);

	return 0;
}

