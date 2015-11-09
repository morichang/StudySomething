/*
点群保存したり
保存した点群読み込んだり
PCLと相互変換したりしたい
*/

#pragma once
#include "../../Kinect2Wrapper/Kinect2Wrapper/Kinect2.h"

#include <fstream>
#include <iostream>

#include <boost/date_time.hpp>
#include <boost/date_time/c_local_time_adjustor.hpp>

#include <pcl/io/pcd_io.h>

#include "KinectPointDef.h"


class KinectPointIO {
private:
	std::string computerName;

public:


private:
	cv::Point3i getPixel(cv::Mat &frame, ColorSpacePoint *colorCoordinates, int depthIndex);

public:

	KinectPointIO();

	//バイナリで書き込む
	template<typename T_n>
	void saveBinary(std::vector<T_n>& pointsList);
	template<typename T_n>
	void saveBinary(std::string filepath, std::vector<T_n>& pointsList);

	//バイナリで読み込む
	void loadBinary(std::string filepath, std::vector<struct float_point>& pointsList);

	//ポイントリストに変換する
	std::vector<struct float_point> convCamera2Points(Kinect2Sensor &kinect);

	//ポイントリストからPCLに変換する
	void convPoints2PCL(std::vector<struct float_point> points, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud);

	//点群を保存する
	void savePoints(Kinect2Sensor &kinect);
	void savePoints(std::string filepath, Kinect2Sensor &kinect);

};