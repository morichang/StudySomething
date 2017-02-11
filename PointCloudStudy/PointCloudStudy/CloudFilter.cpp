#include "CloudFilter.h"

void CloudFilter::Remove_outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(in_cloud);  // 外れ値を除去する点群を入力
	sor.setMeanK(25);  // MeanKを設定
	sor.setStddevMulThresh(1.0);  // 極力ノイズ以外の点は残したい…
	sor.setNegative(false);  // 外れ値を出力する場合はtrueにする
	sor.filter(*out_cloud);  // 出力

	return ;
}