#include "KinectPointIO.h"
template<typename T_n>
void KinectPointIO::saveBinary(std::vector<T_n>& pointsList){
	//**************************************************
	//ファイル名決定
	//**************************************************
	namespace pt = boost::posix_time;
	namespace gg = boost::gregorian;

	// フォーマットの指定
	// facet はストリーム側で自動的に delete される
	auto facet = new pt::time_facet("%Y%m%d_%H%M%S");
	std::stringstream ss;
	ss.imbue(std::locale(std::cout.getloc(), facet));

	// 現在の時刻を取得
	auto now_time = pt::second_clock::local_time();
	ss << computerName << "_";
	ss << "pointcloud_";
	ss << now_time;
	ss << ".k3d";

	saveBinary(ss.str(), pointsList);
}
template<typename T_n>
void KinectPointIO::saveBinary(std::string filepath, std::vector<T_n>& pointsList){
	//**************************************************
	//書き出しファイル設定
	//**************************************************
	std::ofstream ofs(filepath, std::ios_base::out | std::ios_base::binary);
	if(ofs.fail()) {
		std::cout << "ファイルオープン失敗" << std::endl;
		return;
	}

	//**************************************************
	//各情報の出力
	//**************************************************
	for(auto p : pointsList) {
		ofs.write((const char*)&p, sizeof(p));
	}

	ofs.close();

	std::cout << filepath << "を保存しました" << std::endl;
}


void KinectPointIO::savePoints(Kinect2Sensor &kinect){
	auto p = convCamera2Points(kinect);
	saveBinary(p);
}

void KinectPointIO::savePoints(std::string filepath, Kinect2Sensor &kinect){
	auto p = convCamera2Points(kinect);
	saveBinary(filepath, p);
}