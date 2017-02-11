// PointCloudStudy.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"

#include "EditingUtility.h"
#include "FrameMatching.h"
#include "Point2Mesher.h"
#include "Delaunay3d.h"
#include "Camera.h"

#include "DxLib.h"

void do_thread(std::string dind_id)
{
	EditingUtility editUtil(dind_id);
	

	// 使うフレーム抽出
	//editUtil.frameExtract();

	// データ抽出
	//editUtil.extractData();
	//editUtil.createPointCloud();

	editUtil.computeMesh();
}


int _tmain(int argc, _TCHAR* argv[])
{
	//FrameMatching matching;

	std::vector<std::string> dind_id =
	{
		"DIND104", "DIND107", "DIND108", "DIND109", "DIND110", "DIND111", "DIND112"
	};

	std::cout << "処理始めるぞ～!" << std::endl;

	auto startTime = boost::posix_time::microsec_clock::local_time();

	//matching.extractFrameMatching(dind_id);

	//std::thread dind104(do_thread, ref(dind_id[0]));
	//std::thread dind107(do_thread, ref(dind_id[1]));
	//std::thread dind108(do_thread, ref(dind_id[2]));
	//std::thread dind109(do_thread, ref(dind_id[3]));
	//std::thread dind110(do_thread, ref(dind_id[4]));
	//std::thread dind111(do_thread, ref(dind_id[5]));
	std::thread dind112(do_thread, ref(dind_id[6]));

	//dind104.join();
	//dind107.join();
	//dind108.join();
	//dind109.join();
	//dind110.join();
	//dind111.join();
	dind112.join();

	auto endTime = boost::posix_time::microsec_clock::local_time();

	std::cout << "処理時間: " << (endTime - startTime).hours() << "[h] "
		<< (endTime - startTime).minutes() << "[m] "
		<< (endTime - startTime).seconds() << "[s] "
		<< ((endTime - startTime).total_milliseconds() - 
		(((endTime - startTime).hours() * 3600) + ((endTime - startTime).minutes() * 60) + (endTime - startTime).seconds())) << std::endl;

	Sleep(INFINITE);

	return 0;
}