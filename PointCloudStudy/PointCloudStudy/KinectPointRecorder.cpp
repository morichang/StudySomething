#include "KinectPointRecorder.h"
#include "KinectPointDef.h"

void KinectPointRecorder::recordInitialize(std::string filename){
	this->filename = filename;

	ofs.open(filename, std::ios::out | std::ios::binary);
	if(ofs.fail()) {
		std::cout << "ファイルオープン失敗" << std::endl;
		return;
	}

	//初期化
	nowBuffer = 0;
	lastUpdate = boost::posix_time::microsec_clock::local_time();
	targetFPS = 10;
	//バッファサイズは64MB
	maxBufferSize = 1024 * 1024 * 128;

	//ヘッダ書くんだったらここで
}

void KinectPointRecorder::recordClose(){
	std::cout << "閉じます" << std::endl;
	flush();
	ofs.close();
}

void KinectPointRecorder::setData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud){
	if(!ofs.is_open()) {
		std::cout << "おかしいぞコラ" << std::endl;
		return;
	}
	
	//今の時間を得る
	boost::posix_time::ptime nowTime = boost::posix_time::microsec_clock::local_time();

	//保存すべき時刻かどうかチェック
	double interval = 1000.0 / targetFPS;
	if((nowTime - lastUpdate).total_milliseconds() < interval) {
		return;
	}
	//最終更新時刻を更新する
	lastUpdate = nowTime;

	std::list<struct int_point> oneFrame;
	for(auto p : *pointcloud) {
		struct int_point data;
		//スケールをメートル->ミリメートルにする
		data.x = (int)(p.x * 1000);
		data.y = (int)(p.y * 1000);
		data.z = (int)(p.z * 1000);
		data.r = p.b;		//何かおかしい
		data.g = p.g;		//何かおかしい
		data.b = p.r;		//何かおかしい
		//リストに入れる
		oneFrame.push_back(data);
	}

	//リストに入れる
	pointList.push_back(oneFrame);
	timeList.push_back(nowTime);

	//いつ書き込む？
	nowBuffer += oneFrame.size() * sizeof(struct int_point);
	if(nowBuffer >= maxBufferSize) {
		flush();
		nowBuffer = 0;
	}
}

void KinectPointRecorder::flush(){
	//std::cout << "書き込みm@s";

	std::list<boost::posix_time::ptime>::iterator itrTime = timeList.begin();
	std::list<std::list<struct int_point>>::iterator itrPoint = pointList.begin();

	while(itrPoint != pointList.end()) {
		//時刻の書き込み
		ofs.write((const char*)&(*itrTime), sizeof(boost::posix_time::ptime));
		//データ数の書き込み
		size_t dataSize = itrPoint->size();
		ofs.write((const char*)&dataSize, sizeof(size_t));
		//点群の書き込み
		for(auto p : (*itrPoint)) {
			ofs.write((const char*)&p, sizeof(struct int_point));
		}

		//次のデータへ
		itrPoint++;
		itrTime++;
	}

	//データを消す
	timeList.clear();
	pointList.clear();

	//std::cout << "書き込みました" << std::endl;
}