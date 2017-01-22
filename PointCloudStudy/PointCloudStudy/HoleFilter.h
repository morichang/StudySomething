#pragma once

#include "stdafx.h"

#include <omp.h>

class HoleFilter{
private:
	inline double square(double x);
	inline double calc_func(double t);
public:
	void BGR2Lab(cv::Mat &src, cv::Mat &dst, int mat_type);
	void hole_filter(const cv::Mat &src, const cv::Mat &before, cv::Mat &dst, cv::Size kernelSize, double sigma_color, double sigma_space);
	void image_truncate(cv::Mat &src_image, cv::Mat &src_depth, cv::Mat &dst_image, cv::Mat &dst_depth);
};