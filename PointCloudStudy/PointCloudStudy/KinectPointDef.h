#pragma once

#include <opencv2/opencv.hpp>

struct cv_point {
	cv::Point3d point;
	cv::Vec3b r, g, b;
};

struct int_point {
	int x, y, z;
	unsigned char r, g, b;
};

struct float_point {
	float x, y, z;
	unsigned char r, g, b;
};