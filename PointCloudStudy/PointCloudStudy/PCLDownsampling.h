#include <pcl/filters/voxel_grid.h>

class PCLDownsampling {
private:

public:

	void exec(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud){
		// Create the filtering object
		pcl::VoxelGrid<pcl::PointXYZRGB> sor;
		sor.setInputCloud(pointcloud);
		sor.setLeafSize(0.02f, 0.02f, 0.02f);
		sor.filter(*pointcloud);
	}
};