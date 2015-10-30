#include "KinectPointIO.h"

KinectPointIO::KinectPointIO(){
	char szComputerName[256] = {'\0'};
	DWORD dwSize = sizeof(szComputerName) / sizeof(szComputerName[0]);
	GetComputerName(szComputerName, &dwSize);
	computerName = std::string(szComputerName);
}

cv::Point3i KinectPointIO::getPixel(cv::Mat &frame, ColorSpacePoint *colorCoordinates, int depthIndex){
	ColorSpacePoint colorPoint = colorCoordinates[depthIndex];
	//デプス値はカラー画像上ではどこか？(Kinectv1の時と逆のことをしている？)
	int colorX = (int)(floor(colorPoint.X));
	int colorY = (int)(floor(colorPoint.Y));
	int colorIndex = colorX + (colorY * frame.size().width); //ピクセルを特定する
	if((colorX >= 0) && (colorX < frame.size().width) && (colorY >= 0) && (colorY < frame.size().height)) {
		cv::Point3i retval;
		//ここ間違えている可能性高い(BGR?RGB?)
		retval.x = frame.data[(colorY*frame.size().width + colorX) * 3 + 0];
		retval.y = frame.data[(colorY*frame.size().width + colorX) * 3 + 1];
		retval.z = frame.data[(colorY*frame.size().width + colorX) * 3 + 2];
		return retval;
	}
	return cv::Point3i(0, 255, 0);
}

std::vector<struct float_point> KinectPointIO::convCamera2Points(Kinect2Sensor &kinect){
	std::vector<struct float_point> points;

	//**************************************************
	//各情報の出力
	//**************************************************
	auto cameraPoints = kinect.getCameraBuffer();
	for(int i = 0; i < cameraPoints.size(); i++) {
		auto &csp = cameraPoints[i];
		////オーバーフローチェック
		//if(csp.X != csp.X) {
		//	continue;
		//}
		//ピクセルの色取得
		cv::Point3i pixel = getPixel(kinect.getFrame(), kinect.getColorCoordinates(), i);
		//データ設定
		struct float_point p;
		p.x = csp.X;
		p.y = csp.Y;
		p.z = csp.Z;
		p.r = pixel.x;
		p.g = pixel.y;
		p.b = pixel.z;
		//ファイル出力
		points.push_back(p);
	}

	return points;
}

void KinectPointIO::convPoints2PCL(std::vector<struct float_point> points, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud){
	pointcloud->clear();

	for(auto p : points) {
		pcl::PointXYZRGB point;

		point.x = p.x;
		point.y = p.y;
		point.z = p.z;
		point.r = p.b;
		point.g = p.g;
		point.b = p.r;

		//リストにプッシュしていく
		pointcloud->push_back(point);
	}
}