#include "HoleFilter.h"

inline double HoleFilter::square(double x)
{
	return (x * x);
}

inline double HoleFilter::calc_func(double t)
{
	if (t > 0.008856) {
		return std::pow(t, 1 / 3);
	}
	else if (t <= 0.008856) {
		return ((7.787 * t + 16) / 116);
	}
	else {
	}
}

void HoleFilter::BGR2Lab(cv::Mat &src, cv::Mat &dst, int mat_type)
{
	const int H = src.rows;
	const int W = src.cols;
	double delta;
	double L, a, b;
	cv::Vec3b bgr;
	cv::Mat param = (cv::Mat_<double>(3, 3) << 0.412453, 0.357580, 0.1804223, 0.212671, 0.715160, 0.072169, 0.019334, 0.119193, 0.950227);
	cv::Mat labMat(src.size(), CV_64FC3);

	if (mat_type == CV_8UC3) {
		delta = 128.0;
	}
	else {
		delta = 0.0;
	}

	// bgr[0] = blue, bgr[1] = green, bgr[2] = red
	#pragma omp parallel for
	for (int y = 0; y < H; ++y) {
		#pragma omp parallel for
		for (int x = 0; x < W; ++x) {
			cv::Point3d point;
			bgr = src.at<cv::Vec3b>(y, x);
			point.x = (bgr[2] * param.at<double>(0, 0)) + (bgr[1] * param.at<double>(0, 1)) + (bgr[0] * param.at<double>(0, 2));
			point.y = (bgr[2] * param.at<double>(1, 0)) + (bgr[1] * param.at<double>(1, 1)) + (bgr[0] * param.at<double>(1, 2));
			point.z = (bgr[2] * param.at<double>(2, 0)) + (bgr[1] * param.at<double>(2, 1)) + (bgr[0] * param.at<double>(2, 2));
			point.x /= 0.950456;
			point.z /= 1.088754;
			if (point.y > 0.008856) {
				L = (116.0 * std::pow(point.y, 1 / 3) - 16.0);
			}
			else if (point.y <= 0.008856) {
				L = (903.3 * point.y);
			}
			else {
				L = 0.0;
			}
			a = (500.0 * (calc_func(point.x) - calc_func(point.y)) + delta);
			b = (200.0 * (calc_func(point.y) - calc_func(point.z)) + delta);
			//std::cout << L << "," << a << "," << b << std::endl;
			if (mat_type == CV_8UC3) {
				labMat.at<cv::Vec3b>(y, x)[0] = (L * 255.0/100.0);
				labMat.at<cv::Vec3b>(y, x)[1] = (a + 128.0);
				labMat.at<cv::Vec3b>(y, x)[2] = (b + 128.0);
			}
			else if (mat_type == CV_64FC3) {
				labMat.at<cv::Vec3d>(y, x)[0] = L;
				labMat.at<cv::Vec3d>(y, x)[1] = a;
				labMat.at<cv::Vec3d>(y, x)[2] = b;
			}
		}
	}
	dst = labMat.clone();
}

void HoleFilter::hole_filter(cv::Mat &src, cv::Mat &dst, cv::Size kernelSize, double sigma_color, double sigma_space)
{
	const int kernel = kernelSize.area();
	const int H = src.rows;
	const int W = src.cols;
	int x = 0;
	int y;
	const double sigma_c = (2.0 * sigma_color * sigma_color);
	const double sigma_s = (2.0 * sigma_space * sigma_space);
	double n = 0.0, d = 0.0;
	double P = 0.0, N = 0.0, grad = 0.0;
	cv::Vec3b Lab, LabAlpha;

	auto starttime = boost::posix_time::microsec_clock::local_time();

	#pragma omp parallel for
	for (y = 0; y < H; ++x) {
		n = 0;
		d = 0;
		Lab = src.at<cv::Vec3b>(y, x);
		if (dst.at<UINT16>(y, x) == 0) {
			#pragma omp parallel for
			for (int i = -kernel; i <= kernel; ++i) {
				#pragma omp parallel for
				for (int j = -kernel; j <= kernel; ++j) {
					if (((y + i >= 0) && (y + i < H)) && ((x + j >= 0) && (x + j<W))) {
						LabAlpha = src.at<cv::Vec3b>(y + i, x + j);
						P = std::exp(-((i * i + j * j) / (sigma_c)));
						grad = square(Lab[0] - LabAlpha[0]) + square(Lab[1] - LabAlpha[1]) + square(Lab[2] - LabAlpha[2]);
						N = std::exp(-(sqrt(grad) / (sigma_s)));
						if (dst.at<UINT16>(y + i, x + j) != 0) {
							n += dst.at<UINT16>(y + i, x + j) * (P * N);
							d += (P * N);
						}
					}
					else {
						LabAlpha = 0;
						P = std::exp(-((i * i + j * j) / (sigma_c)));
						grad = square(Lab[0] - LabAlpha[0]) + square(Lab[1] - LabAlpha[1]) + square(Lab[2] - LabAlpha[2]);
						N = std::exp(-(sqrt(grad) / (sigma_s)));
						n += 0 * (P * N);
						d += (P * N);
					}
					
				}
			}
			dst.at<UINT16>(y, x) = static_cast<UINT16>(n / d);
		}
		if (!(x < W)) {
			++y;
			x = 0;
		}
	}

	auto endtime = boost::posix_time::microsec_clock::local_time();

	std::cout << "ŽžŠÔ:" << (endtime - starttime) << std::endl;
}

void HoleFilter::image_truncate(cv::Mat &src_image, cv::Mat &src_depth, cv::Mat &dst_image, cv::Mat &dst_depth)
{
	int H;

	if (!(src_image.rows == src_depth.rows)) {
		H = 1080;
	}
	else {
		H = src_image.rows;
	}
	
	if (!(src_depth.type() == CV_16UC1) && !(src_image.type() == CV_8UC3)) {
		std::cerr << "Type‚ª‚¨‚©‚µ‚¢" << std::endl;
	}
	else {
		dst_image = cv::Mat::zeros(1080, 1440, src_image.type());
		dst_depth = cv::Mat::zeros(1080, 1440, src_depth.type());
	}

	for (int y = 0; y < H; ++y) {
		for (int x = 0; x < 1440; ++x){
			dst_image.at<cv::Vec3b>(y, x)[0] = src_image.at<cv::Vec3b>(y, x + 240)[0];
			dst_image.at<cv::Vec3b>(y, x)[1] = src_image.at<cv::Vec3b>(y, x + 240)[1];
			dst_image.at<cv::Vec3b>(y, x)[2] = src_image.at<cv::Vec3b>(y, x + 240)[2];
		}
	}

	for (int y = 0; y < H; ++y) {
		for (int x = 0; x < 1440; ++x){
			dst_depth.at<UINT16>(y, x) = src_depth.at<UINT16>(y, x + 240);
		}
	}
}