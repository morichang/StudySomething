#include "PCLUtility.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLUtility::createKinectPointCloud(cv::Size depthSize){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pointcloud->width = static_cast<uint32_t>(depthSize.width);
	pointcloud->height = static_cast<uint32_t>(depthSize.height);
	pointcloud->is_dense = false;

	return pointcloud;
}

void PCLUtility::convKinectDataToPointCloudXYZRGB(Kinect2Sensor &kinect, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud){
	pointcloud->clear();

	//pointcloud->width = 640;
	//pointcloud->height = 480;

	//カラー画像の取得(ポインタ)
	cv::Mat test = kinect.getFrame();
	unsigned char *colorImagePointer = test.data;

	//深度画像全ピクセル分のループ
	for(int y = 0; y < kinect.getDepthSize().height; y++) {
		for(int x = 0; x < kinect.getDepthSize().width; x++) {
			//点群の中の1個の点
			pcl::PointXYZRGB point;

			int depth_target = y * kinect.getDepthSize().width + x; //depthの注目画素
			//**************************************************************************
			//1. 深度画像の1点の深度値を得る
			//深度値の取得
			//**************************************************************************
			unsigned int depth = kinect.getDepthBuffer()[depth_target];

			//**************************************************************************
			//2. 深度画像の1点が3次元座標ではどこかを得る
			//点の3次元位置を得る
			//**************************************************************************
			CameraSpacePoint skeletonPoint = kinect.getCameraBuffer()[depth_target];
			//深度値のセット
			point.x = skeletonPoint.X;
			point.y = skeletonPoint.Y;
			point.z = skeletonPoint.Z;

			//**************************************************************************
			//3. 深度画像の1点がカラー画像ではどこかを得る
			//点のカラー情報を得る
			//**************************************************************************
			cv::Point colorPoint = kinect.getMapper()->getColorUVFromDepthUV(x, y);

			//深度カメラのピクセルがカラー画像の範囲外にあるかもしれない
			if(0 <= colorPoint.x && colorPoint.x < kinect.getColorSize().width && 0 <= colorPoint.y && colorPoint.y < kinect.getColorSize().height) {
				unsigned int index = (colorPoint.y * kinect.getColorSize().width + colorPoint.x) * 3; //ARGB
				//カラー値のセット
				point.r = colorImagePointer[index + 2];
				point.g = colorImagePointer[index + 1];
				point.b = colorImagePointer[index + 0];

				//リストにプッシュしていく
				pointcloud->push_back(point);
			}
		}
	}
}