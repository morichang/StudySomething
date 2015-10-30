#include <fstream>
#include <boost/date_time.hpp>

#include <pcl/io/pcd_io.h>

#include <list>

#include "KinectPointDef.h"
class KinectPointRecorder {
private:

	std::string filename;				//書き込むファイル名
	std::ofstream ofs;

	std::list<boost::posix_time::ptime> timeList;		//記録した時間
	std::list<std::list<struct int_point>> pointList;	//記録したテングン

	void flush();

	size_t maxBufferSize;	//最大バッファ数
	size_t nowBuffer;		//今のバッファ

	boost::posix_time::ptime lastUpdate;	//最終更新時刻
	double targetFPS;						//fps
public:

	void recordInitialize(std::string filename);

	void setData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud);

	void recordClose();

};