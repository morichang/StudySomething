#include <pcl/filters/voxel_grid.h>

class PCLDownsampling {
private:

public:

	void exec(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud){
		// Create the filtering object
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(pointcloud);
		sor.setLeafSize(0.035f, 0.035f, 0.035f);
		sor.filter(*pointcloud);
	}
};