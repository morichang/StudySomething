#include "CloudFillter.h"

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